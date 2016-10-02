
========
Protocol
========

This document describes the protocol used to communicate from Apollo 
to ROS. What topic names are used and how relays to ROS topics can
be created.

The bridge is a ROS module that connects the ROS network with the Apollo
network. The bridge can be controlled by sending config messages to
it from the Apollo network. The bridge will send status messages back
(into the Apollo network). The bridge can also be queried for certain
information, and it acts as a proxy for ROS service requests.

Entities in the system:
    
  * An application that wants to talk to the robot
  * The Apollo broker
  * The ROS broker
  * A bridging ROS module: bridge
  * The ROS module (that will usually control a robot)

We will assume that the ROS interface of the robot is defined a priori
in terms of ROS topics that it will listen and publish to, as well as
the datatypes of these topics.


Topic names
-----------

.. Note::
    The bridge module can be given an identifier `ID`, which is useful
    when multiple ROS brokers are connected to one Apollo broker. An
    identifier may only consist of alphanumeric characters. If an
    identifier specified, all Apollo topics will be prefixed by `ID_`,
    i.e. if the bridge identifier is 'test' and the topic name is
    'testtopic', then the topic will be::
        
        apollo_topic = '/topic/test_testtopic'
    
    If no identfier is specified (i.e. an empty string), the underscore
    will be omitted, and topics will have the usual form::
        
        apollo_topic = '/topic/testtopic'

The bridge subscribes to three Apollo topics on start-up:
`ID_bridge_config`, `ID_bridge_info`, and `ID_bridge_service`. It
publishes on `ID_bridge_status`, `ID_bridge_info` and
`ID_bridge_service`. The use of these topics are explained in the
sections below.

The bridge is responsible for making connections between the Apollo
network and ROS network, by relaying two topics from either network.
The messages send over these network are structures of a specific data
type (see below).

The names of the corresponding Apollo and ROS topics are very similar,
but differ slightly due to constraints in what names can be used in
both networks. Given a topic name as used in the bridge config or status
messages, the Apollo and ROS topic names can be constructed as follows::

    ros_topic = '/' + TOPIC_NAME
    apollo_topic = '/topic/' + TOPIC_NAME.replace('/', '.')



Data structure
--------------

Relays can only be set up for data types that the bridge knows how to
translate, i.e. from and to xml on the Apollo side, and the actual
datatypes on the ROS side. Assuming that a robot control package
subscribes to a topic with the following datatype::

    struct:
        x: float
        y: float
        color:
            r: int
            g: int
            b: int

i.e., the callback responding to the topic messages assumes a struct
with fields 'x' and 'y' with float values, and a field 'color' with
fields 'r', 'g' and 'b' with integer values. This format is specified
in advance in ROS.

The messages on the corresponding topic must then have the following
format::

    <data>
     <x type="float">1.0</x>
     <y type="float">2.0</y>
     <color type="struct>
      <r type="int">64</r>
      <g type="int">128</g>
      <b type="int">255</b>
     </color>
    </data>

Such messages are used to communicate the data to and from ROS, but also
to communicate the structure of the data (in which case all elements 
have default values).

.. Note::
    Functions to convert such XML messages to a Python dict (and back)
    are available in the robotutils package.



The config topic
----------------

The bridge is subscribed to the `bridge_config` topic. It is used to
configure the bridge, e.g. setting up relays (see below). A message
sent on this topic must have the following format::
    
    <config>
        <relay publisher="ros" name=TOPIC_NAME />
        <relay publisher='apollo' name=TOPIC_NAME />
    </config>

As can be seen, one can put multiple subscriptions in one config message.

This first request instructs bridge to subscribe to a ROS topic and
relay the received message to the corresponding Apollo topic. The second
message does the reverse.

.. Note::
    Note that we are specifying ROS topics, and thus the bridge
    identifier is not included in the topic name!

.. Note::
    Other configuration tags may be defined in the future.


The status topic
----------------

The bridge publishes on the `bridge_status` topic. It is used to publish
bridge status, and to react to config messages. For example, the bridge
answers with 'ok' or 'failed' to notify the status of relays it was
asked to set up::

    <status>
        <relay publisher="ros" name=TOPIC_NAME>ok</relay>
        <relay publisher="apollo" name=TOPIC_NAME>failed</relay>
        <error>invalid topic name</error>
    </status>



The info topic
--------------

The bridge is subscribed, and publishes to the `bridge_info` topic.
It is used to request information from the bridge. Each
request is represented with one request tag. A message on this topic
must thus have the following format::
    
    <info>
        <request type='msg-structure' name=TOPIC_NAME />
        <request type='srv-structure' name=TOPIC_NAME />
        <request type='list-pubs-subs-services' />
    </info>

The 'type' attribute specifies the type of information that is requested.
Depending on this type, additional attributes may need to be specified.
This example shows requesting the stucture of the data for a certain 
topic and for a service, as well as requesting a list of all current
ROS topics ans services.

The info topic is also used for sending the response to the information 
request. Such a response may look like::
    
    <info>
        <response type='msg-structure' name=TOPIC_NAME>
            ... 
        </response>
        <response type='srv-structure' name=TOPIC_NAME>
            <request-structure>
            ... 
            </request-structure>
            <response-structure>
            ... 
            </response-structure>
        </response>
        <response type='topic-list'>
            <pub-topic>foo/bar</pub-topic>
            <sub-topic>bar/foo</sub-topic>
            <service>service1</service>
            etc.
        </response>
    </info>

The triple dots ``...`` indicate the inclusion of a data structure.


The service topic
-----------------

The bridge is subscribed, and publishes to the `bridge_service` topic.
It is used to perform service calls to ROS nodes. Many ROS nodes have
services available that can be called. This functionality is available
via the bridge.

A service request message has the following form (the tripple dots
represents a data structure with the service arguments)::
    
    <service>
        <request name=SERVICE_NAME>
            ... 
        </request>
    </service>

The bridge answers on the same topic (the tripple dots represents a
data structure with the return value of the service)::

    <service>
        <response name=SERVICE_NAME>
            ... 
        </response>
    </service>

