#!/usr/bin/env python

from __future__ import print_function

import sys
import traceback
import importlib

import xml.etree.ElementTree as et

import roslib
import rospy, rostopic, rosservice, rosgraph
import stomp

from stomp.listener import ConnectionListener
from robotutils import get_config, set_config, xml2dict
from xml_conversion import xml_to_rosobject, rosobject_to_xml

APOLLO_PREFIX = '/topic/'
ROS_PREFIX = '/'

APOLLO_STATUS_TOPIC = 'bridge_status'
APOLLO_CONFIG_TOPIC = 'bridge_config'
APOLLO_INFO_TOPIC = 'bridge_info'
APOLLO_SERVICE_TOPIC = 'bridge_service'

RELAY_OK_TEXT = 'ok'
RELAY_FAIL_TEXT = 'failed'

MSG_TYPE_UNKNOWN_TEXT = 'unknown'
SRV_TYPE_UNKNOWN_TEXT = 'unknown'


def print_exception(self, when=''):
    # get traceback and store
    type, value, tb = sys.exc_info()
    # Show traceback
    tblist = traceback.extract_tb(tb)
    list = traceback.format_list(tblist)  # (tblist[2:])
    list.extend(traceback.format_exception_only(type, value))
    # print
    rospy.logerr("ERROR %s:", when)
    for i in list:
        rospy.logerr(i)


class Bridge:
    """
    Implementation of the bridge functionality.
    
    This class implements a bridge between the ROS broker and an Apollo
    broker. The connection is made by subscribing and publishing on
    specific topics on both network, and relaying the messages on those
    topics. See documentation for further details.
    
    Parameters
    ----------
    bridge_id : str
        Unique identifier for this bridge on the Apollo network. Only
        relevant if multiple ROS brokers are connected to one Apollo
        network. May be an empty string if no identifier is required.
    apollo_login : dict
        A dict with keys 'host_and_ports' and 'auto_content_length = False'. See
        stomp.py documentation for details.
        The auto_content_length is set to false to force TextMessages instead of
        BytesMessages that are being send
        See: https://github.com/jasonrbriggs/stomp.py/issues/82
    """

    def __init__(self, bridge_id, apollo_login):
        msg_class, _, _ = rostopic.get_topic_class("/asap/animation")
        # Set bridge id
        if type(bridge_id) is not str:
            raise ValueError('Invalid bridge identifier')

        if bridge_id == '':
            self._id = ''
        else:
            if bridge_id.isalnum():
                self._id = bridge_id + '_'  # _ separates from real topic name
            else:
                raise ValueError('Invalid bridge identifier')

        # id counter used for subscribing to apollo topics (STOMP versions > 1.0 need a unique ID for each topic)
        self._idCounter = 1

        # keep track of all subscribed ROS and Apollo topics so we can't accidentally double-subscribe!
        self._ros_topics = []
        self._apollo_topics = []

        # Connect to Apollo network
        self._apollo = stomp.Connection(**apollo_login)

    def start(self):
        """ Start the bridge. """

        # Start connection to Apollo
        self._apollo.start()
        self._apollo.connect(username='admin', passcode='password')
        self._bind_to_system_topics()

        # Load bridge configuration if available
        bridge_config = get_config('bridge')
        if bridge_config.has_key('config_xml'):
            self._parse_config_message(bridge_config['config_xml'])

    def stop(self):
        """ Stop the bridge. """
        self._apollo.disconnect()

    def _bind_to_system_topics(self):
        def bind_listener(name, topic, callback):
            apollo_topic = APOLLO_PREFIX + self._id + topic

            class AbstractListener(ConnectionListener):
                def __init__(self):
                    ConnectionListener.__init__(self)

                def on_error(self, headers, message):
                    rospy.logerr('Error in abstract listener:\n%s\n%s', headers, message)

                def on_message(self, headers, message):
                    if headers['destination'] == apollo_topic:
                        callback(message)

            listener = AbstractListener()
            self._apollo.set_listener(name, listener)
            self._apollo.subscribe(destination=apollo_topic, ack='auto', id=self._idCounter)

            self._idCounter += 1

        # Bind listeners to system topics
        bind_listener(
            'config', APOLLO_CONFIG_TOPIC, self.on_config_message)
        bind_listener(
            'info', APOLLO_INFO_TOPIC, self.on_info_message)
        bind_listener(
            'service', APOLLO_SERVICE_TOPIC, self.on_service_message)

    def _create_apollo_publisher(self, topic, msg_class):
        """
        Creates a relay by subcribing to an Apollo topic, and publishing
        the received data to a ROS topic with the same name.
        
        Parameters
        ----------
        topic : string
            String identifier of the topic to be linked.
        msg_class : class
            ROS message class
        """

        # Only create new publisher/listener if not already existant
        if topic in self._apollo_topics:
            return
        else:
            self._apollo_topics.append(topic)

        # Create ROS publisher for the topic
        d = ROS_PREFIX + topic
        pub = rospy.Publisher(d, msg_class, queue_size=1)

        # Create Apollo listener object for the topic
        bridge_id = self._id

        class ApolloListener(ConnectionListener):
            def __init__(self):
                ConnectionListener.__init__(self)

            def on_error(self, headers, message):
                rospy.logerr('Error in Apollo listener:\n%s', message)

            def on_message(self, headers, message):
                topic_name = \
                    headers['destination'][len(APOLLO_PREFIX + bridge_id):]
                topic_name = topic_name.replace('.', '/')
                if topic_name == topic:
                    rospy.logdebug('Got Apollo message:\n%s\nWith headers:\n%s', message, headers)
                    try:
                        obj = xml_to_rosobject(message, msg_class)
                        pub.publish(obj)
                        rospy.logdebug('Published message:\n%s\n', obj)
                    except Exception as err:
                        rospy.logerr("Error publishing to ROS:\n%s\n%s", message, err)  # Short
                        # print_exception("publishing to ROS")  # Detailed

        # Bind callback to topic
        if not self._apollo.get_listener(topic):
            listener = ApolloListener()
            self._apollo.set_listener(topic, listener)
            d = APOLLO_PREFIX + self._id + topic.replace('/', '.')
            self._apollo.subscribe(destination=d, ack='auto', id=self._idCounter)
            self._idCounter += 1

    def _create_ros_publisher(self, topic, msg_class):
        """
        Creates a relay by subscribing to a ROS topic, and publishing the
        received date to an Apollo topic with the same name.
        
        Parameters
        ----------
        topic : string
            String identifier of the topic to be linked.
        msg_class : class
            ROS message class
        """

        # Only create new publisher/listener if not already existant
        if topic in self._ros_topics:
            return
        else:
            self._ros_topics.append(topic)

            # Create ROS listener object for the topic
        bridge_id = self._id
        apollo = self._apollo

        class ROSListener:

            def on_error(self, headers, message):
                rospy.logerr('Error in ROS listener:\n%s', message)

            def on_message(self, data):
                d = APOLLO_PREFIX + bridge_id + topic.replace('/', '.')
                try:
                    apollo.send(body=rosobject_to_xml(data), destination=d)
                except Exception as err:
                    # rospy.logerr("Error publishing to Apollo:\n%s", err)
                    print_exception("Publishing to Apollo")  # Detailed

        # Bind callback to topic
        listener = ROSListener()
        d = ROS_PREFIX + topic
        rospy.Subscriber(d, msg_class, listener.on_message)

    def on_config_message(self, message):
        try:
            return self._parse_config_message(message)
        except Exception:
            # We should catch all errors while parsing, but just in case ...
            print_exception('Parsing config message.')

    def on_info_message(self, message):
        try:
            return self._parse_info_message(message)
        except Exception:
            # We should catch all errors while parsing, but just in case ...
            print_exception('Parsing info message.')

    def on_service_message(self, message):
        try:
            return self._parse_service_message(message)
        except Exception:
            # We should catch all errors while parsing, but just in case ...
            print_exception('Parsing service message.')

    def _parse_config_message(self, message):
        rospy.logdebug('Got config message:\n%s', message)
        """ Parses a config message. """

        # Set up status message
        status_topic = APOLLO_PREFIX + self._id + APOLLO_STATUS_TOPIC
        status = et.Element('status')

        # Try to parse the message
        try:
            root = et.fromstring(message)
        except et.ParseError as e:
            rospy.logerr(str(e))
            et.SubElement(status, 'error').text = str(e)
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Check if a config message was received
        if root.tag != 'config':
            e = 'Invalid message on config topic'
            rospy.logerr(e)
            et.SubElement(status, 'error').text = e
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Process relays
        for relay in root.findall('relay'):
            # Try to get publisher
            publisher = relay.get('publisher')
            if publisher is None:
                e = 'no publisher supplied for relay'
                rospy.logerr(e)
                et.SubElement(status, 'error').text = e
                continue

            # Try to get topic name
            topic_name = relay.get('name')
            if topic_name is None:
                e = 'no name supplied for relay'
                rospy.logerr(e)
                et.SubElement(status, 'error').text = e
                continue

            # Prepare status message
            relay_status = et.SubElement(status, 'relay')
            relay_status.set('publisher', publisher)
            relay_status.set('name', topic_name)

            # Try to obtain the message class for the type
            ros_topic = ROS_PREFIX + topic_name
            try:
                msg_class, _, _ = rostopic.get_topic_class(ros_topic)
            except rostopic.ROSTopicException as re:
                e = 'Invalid topic name or unknown message format \'%s\'' % ros_topic
                rospy.logerr(e)
                rospy.logerr(re)
                et.SubElement(status, 'error').text = e
                relay_status.text = RELAY_FAIL_TEXT
                continue

            # Check message class
            if msg_class is None:
                e = 'No class found for topic \'%s\'' % ros_topic
                rospy.logerr(e)
                et.SubElement(status, 'error').text = e
                relay_status.text = RELAY_FAIL_TEXT
                continue

            # Create relays
            if publisher.lower() == 'ros':
                rospy.logdebug('Creating ROS publisher on topic \'%s\' (%s)' % (
                    topic_name,
                    msg_class))
                relay_status.text = RELAY_OK_TEXT
                self._create_ros_publisher(topic_name, msg_class)
            elif publisher.lower() == 'apollo':
                rospy.logdebug('Creating Apollo publisher on topic \'%s\' (%s)' % (
                    topic_name,
                    msg_class))
                relay_status.text = RELAY_OK_TEXT
                self._create_apollo_publisher(topic_name, msg_class)
            else:
                e = 'Invalid publisher \'%s\'' % publisher
                et.SubElement(status, 'error').text = e
                relay_status.text = RELAY_FAIL_TEXT
                continue

        # TODO: other configuration tags that need to be parsed?

        # Send status message
        if len(status):
            self._apollo.send(body=et.tostring(status), destination=status_topic)

    def _parse_info_message(self, message):
        """ Parses an info message. """

        # Set up info message
        info_topic = APOLLO_PREFIX + self._id + APOLLO_INFO_TOPIC
        info = et.Element('info')

        # Set up status message
        status_topic = APOLLO_PREFIX + self._id + APOLLO_STATUS_TOPIC
        status = et.Element('status')

        # Try to parse the message
        try:
            root = et.fromstring(message)
        except et.ParseError as e:
            et.SubElement(status, 'error').text = str(e)
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Check if an info message was received
        if root.tag != 'info':
            e = 'Invalid message on info topic'
            et.SubElement(status, 'error').text = e
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Process requests
        for request in root.findall('request'):
            request_type = request.get('type')
            if request_type is None:
                e = 'Missing request type'
                et.SubElement(status, 'error').text = e
                continue
            else:
                request_type = request_type.lower()
                rospy.logdebug('Answering request of type \'%s\'', request_type)

            if request_type == 'msg-structure':
                # Try to get topic name
                topic_name = request.get('name')
                if topic_name is None:
                    e = 'No name supplied for request'
                    et.SubElement(status, 'error').text = e
                    continue

                # Try to obtain the message class for the topic
                ros_topic = ROS_PREFIX + topic_name
                try:
                    msg_class, _, _ = rostopic.get_topic_class(ros_topic)
                except rostopic.ROSTopicException as re:
                    rospy.logerr(re)
                    e = 'Invalid topic name \'%s\'' % ros_topic
                    et.SubElement(status, 'error').text = e
                    continue

                # Get message class structure
                if msg_class is None:
                    e = 'No class found for topic \'%s\'' % ros_topic
                    et.SubElement(status, 'error').text = e
                    continue
                else:
                    response = et.SubElement(info, 'response')
                    response.set('type', request_type)
                    response.set('name', topic_name)

                    obj = msg_class()
                    struct = rosobject_to_xml(obj, as_string=False)
                    for f in struct:
                        response.append(f)
            elif request_type == 'srv-structure':
                # Try to get service name
                service_name = request.get('name')
                if service_name is None:
                    e = 'No name supplied for request'
                    et.SubElement(status, 'error').text = e
                    continue

                # Try to obtain the service class for the service
                ros_service = ROS_PREFIX + service_name
                try:
                    service_class = \
                        rosservice.get_service_class_by_name(ros_service)
                except rosservice.ROSServiceException as re:
                    rospy.logerr(re)
                    e = 'Invalid service name \'%s\'' % service_name
                    et.SubElement(status, 'error').text = e
                    continue

                # Get service class structure
                if service_class is None:
                    e = 'No class found for service \'%s\'' % service_name
                    et.SubElement(status, 'error').text = e
                    continue
                else:
                    response = et.SubElement(info, 'response')
                    response.set('type', request_type)
                    response.set('name', service_name)

                    # Request structure
                    req_elem = et.SubElement(response, 'request-structure')
                    req_obj = service_class._request_class()
                    req_struct = rosobject_to_xml(req_obj, as_string=False)
                    for f in req_struct:
                        req_elem.append(f)

                    # Response structure
                    resp_elem = et.SubElement(response, 'response-structure')
                    resp_obj = service_class._response_class()
                    resp_struct = rosobject_to_xml(resp_obj, as_string=False)
                    for f in resp_struct:
                        resp_elem.append(f)
            elif request_type == 'list-pubs-subs-services':
                # Build response
                response = et.SubElement(info, 'response')
                response.set('type', request_type)
                try:
                    self._get_list_pubs_subs_services(response)
                except Exception:
                    e = 'Error getting lists'
                    et.SubElement(status, 'error').text = e
                    continue
            else:
                e = 'Invalid request type \'%s\'' % request_type
                et.SubElement(status, 'error').text = e
                continue

        # Send info message
        if len(info):
            self._apollo.send(body=et.tostring(info), destination=info_topic)

        # Send status message
        if len(status):
            self._apollo.send(body=et.tostring(status), destination=status_topic)

    def _parse_service_message(self, message):
        """ Parses an service message. """

        # Set up service message
        service_topic = APOLLO_PREFIX + self._id + APOLLO_SERVICE_TOPIC
        service = et.Element('service')

        # Set up status message
        status_topic = APOLLO_PREFIX + self._id + APOLLO_STATUS_TOPIC
        status = et.Element('status')

        # Try to parse the message
        try:
            root = et.fromstring(message)
        except et.ParseError as e:
            et.SubElement(status, 'error').text = str(e)
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Check if a service message was received
        if root.tag != 'service':
            e = 'Invalid message on service topic'
            et.SubElement(status, 'error').text = e
            self._apollo.send(body=et.tostring(status), destination=status_topic)
            return

        # Process requests
        for request in root.findall('request'):
            # Try to get service name
            service_name = request.get('name')
            if service_name is None:
                e = 'No name supplied for request'
                et.SubElement(status, 'error').text = e
                continue
            else:
                rospy.logdebug('Service request \'%s\'', service_name)

            # Try to obtain the service class for the service
            ros_service = ROS_PREFIX + service_name
            try:
                service_class = \
                    rosservice.get_service_class_by_name(ros_service)
            except rosservice.ROSServiceException as re:
                rospy.logerr(re)
                e = 'Invalid service name \'%s\'' % service_name
                et.SubElement(status, 'error').text = e
                continue

            # Check service class
            if service_class is None:
                e = 'No class found for service \'%s\'' % service_name
                et.SubElement(status, 'error').text = e
                continue

            # Check if service is available
            args = xml2dict(request)
            try:
                rospy.wait_for_service(ros_service, timeout=120)
            except rospy.ROSException as re:
                rospy.logerr(re)
                e = 'Service unavailable \'%s\'' % service_name
                et.SubElement(status, 'error').text = e
                continue

            # Try to call service
            service_proxy = rospy.ServiceProxy(ros_service, service_class)
            try:
                res = service_proxy(**args)
            except rosservice.ROSSerializationException as re:
                rospy.logerr(re)
                e = 'Service call failed \'%s\'' % service_name
                et.SubElement(status, 'error').text = e
                continue

            # Process result
            response = et.SubElement(service, 'response')
            response.set('name', service_name)

            res_xml = rosobject_to_xml(res, as_string=False)
            for r in res_xml:
                response.append(r)

        # Send service message
        if len(service):
            self._apollo.send(body=et.tostring(service), destination=service_topic)

        # Send status message
        if len(status):
            self._apollo.send(body=et.tostring(status), destination=status_topic)

    def _get_list_pubs_subs_services(self, element):
        """ Obtain list of publishers, subscribers and services. """

        # Get lists from ROS
        master = rosgraph.Master('/rostopic')
        pubs, subs, services = master.getSystemState()

        # Add to element
        for name, nodes in pubs:
            sub = et.SubElement(element, 'pub-topic')
            sub.set('isrelayed', str('/apollobridge' in nodes))
            sub.text = name
        for name, nodes in subs:
            sub = et.SubElement(element, 'sub-topic')
            sub.set('isrelayed', str('/apollobridge' in nodes))
            sub.text = name
        for name, nodes in services:
            sub = et.SubElement(element, 'service')
            sub.text = name


## Entry point
if __name__ == '__main__':
    # TODO: Change this to reflect the desired configuration!
    # Use the set_config function in the robotutils package to write a
    # configuration file.
    bridge_config = get_config('bridge',
                               bridge_id='ramsocialrobot',
                               host_and_ports=[('bobv.student.utwente.nl', 61613,)],
                               user='admin',
                               passcode='password')

    # Apollo network configuration
    apollo_login = {'host_and_ports': bridge_config['host_and_ports'], 'auto_content_length': False}

    # Start the bridge module
    rospy.init_node('apollobridge')
    bridge = Bridge(bridge_config['bridge_id'], apollo_login)
    bridge.start()
    rospy.loginfo('Bridge started!')

    # Keep alive
    rospy.spin()

    # Close connection
    bridge.stop()
    rospy.logdebug('Bridge stopped!')
