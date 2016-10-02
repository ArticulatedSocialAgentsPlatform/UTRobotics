#!/usr/bin/env python3
# Copyright (C) 2013, Science Applied v.o.f.

"""
The generic control app allows the user to connect to a certain ROS
topic or ROS service, after which he is presented with a graphical
interface that allows the user to control a robot using sliders. The
user can also edit the template XML message directly to control a ROS
module. In addition, scalar values in messages can be associated with
faders of the BFC2000.


**Making a connection**

On startup, the user is presented with a simple UI that allows the user
to select the name of the bridge (the demo configuration uses 'test'),
and the topic/service name to connect with. Using the drop-down menu,
all available topics for the selected bridge can be seen (select the bridge
id field and press enter to refresh the list). The input fields also
supports autocompletion for easy selection of a topic or service. For
clarity each name is prepended with either 'pub:', 'sub:' or 'service:'.

When a selection is made, hit the connect button to proceed. At this
point, the application will communicate with the bridge and ask it
to set up a relay for the selected topic. Next, it will ask the bridge
what the structure of the messages for the selected topic should be.
This then used to create a template xml message, and to create a
structured visual interface corresponding to that message. 
(The details for services differ slightly, but the idea is mostly the
same.)


**Subscribing to a topic**

When subscribed to a topic, you cannot edit the UI elements. You can
only observe how the values change.


**Publishing to a topic**

There are two ways to modify a message: 1) by editing the raw xml; 2)
by using the UI interface. In the first case, make sure the 'raw xml'
checkbox is checked. Changes made to the xml are reflected in the UI when
the checkbox is disabled. Similarly, changes in the UI instantly update
the xml.

A new message is send to the bridge as soon as a value is changed using
the UI. Further a message is send every 500 ms.


**Using a service**

A service follows the request-reply pattern. While publishing on a topic
represents sending a stream of messages, with a service you typically
send one message (a request) and then receive a message back (reply).
Therefore the interface for a service has an additional button to send
the request, and a field that displays the received reply.


**Connecting the motor fader**

Faders (i.e. sliders) and encoders (i.e. turning knobs) can be associated
to float and int values of a message. To do so, click on the little arrow
indicator to the left of the corresponding slider or spinbox. Then select
'Associate BCF-2000 fader' (or 'encoder'). You will be asked to select the
fader number, which should be between 1 and 8. Use 0 to disassociate the 
fader.

When a shader or encoder is associated, you can use the BCF-2000 controls
to directly control the value that it is associated with. When publishing
on a topic, this means direct control of a ROS module. When subsribing to
a topic, this means the BCF-2000 controls follow the values as published
by the ROS module. To determine the range of the control, use the same
menu, and select 'Set Range'.

Note that the BCF2000 motor fader must be connected and that the correct port
is selected via the config (this can differ per computer).

"""

"""
This module defines the main interface. It also handles setting up a
connection. When that succeeds, a TopicDisplay is created which handles
things further.
"""

import sys
import os
import time
import math
import xml.etree.ElementTree as et

import stomp

from robotutils.qt import QtGui, QtCore
from robotutils.config import get_config, set_config
from robotutils import xml2dict, dict2xml

from messagedisplay import PubDisplay, SubDisplay, ServiceDisplay


# Get login
LOGIN = get_config('apollo',    host_and_ports=[('localhost', 61613)])


MSG_CONNECT = """
<config>
    <relay publisher="{publisher}" name="{name}" />
</config>
"""

MSG_INFO = """
<info>
 <request type="{type}" name="{name}" />
</info>
"""

MSG_TOPIC_LIST = """
<info>
 <request type="list-pubs-subs-services" />
</info>
"""



class GenericController(QtGui.QMainWindow):
    """ Main widget for the generic controller.
    """
    
    structReceived = QtCore.Signal(tuple, str, str)
    
    def __init__(self):
        QtGui.QMainWindow.__init__(self, None)
        
        # Connect
        self._bridge_id = 'test'
        self._subscribed_bridge_ids = []
        self._stomp_connect()
        
        # Init our properties
        self.setTabPosition(QtCore.Qt.AllDockWidgetAreas,QtGui.QTabWidget.North)
        self.setDockOptions(
                QtGui.QMainWindow.AllowNestedDocks
            |  QtGui.QMainWindow.AllowTabbedDocks
            #|  QtGui.QMainWindow.AnimatedDocks
            )
        
        # Create toolbar
        self._toolbar = self.addToolBar('Topic select')
        self._toolbar.setFloatable(False)
        self._toolbar.setAllowedAreas(QtCore.Qt.TopToolBarArea | QtCore.Qt.BottomToolBarArea)
        
        # Create topic select widget
        self._topicSelect = TopicSelect(self)
        #self.setCentralWidget(self._topicSelect)
        self._toolbar.addWidget(self._topicSelect)
        
        # Initialize list of topic displays
        self._topicDisplays = []
        
        # Connect signals
        self.structReceived.connect(self.onStructReceived)
    
    
    def _stomp_connect(self):
        # Create connection
        self._conn = stomp.Connection(**LOGIN)
        # Add listener
        self._conn.set_listener('generic_control', self)
        # Start the connection
        self._conn.start()
        self._conn.connect(username='admin',passcode='password')
    
    
    def onStructReceived(self, info, xml1, xml2=''):
        # Get class to create
        klass = {'pub': PubDisplay, 'sub': SubDisplay, 'service': ServiceDisplay}[info[0]]
        
        # Instantiate it
        w = klass(self, self._conn, self._bridge_id)
        w.build(info, str(xml1), str(xml2))
        self._topicDisplays.append(w)
        
        # Create dock widget to put it in
        dock = TopicDockWidget(self)
        dock.setWindowTitle('%s (%s)' % (info[1], info[0]))
        dock.setWidget(w)
        # Add the dockwidget
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)
    
    
    def makeConnection(self, type, name):
        
        # Store so we can filter the response
        self._connect_request = type, name
        #print('CONNECT', *self._connect_request)
        
        # Construct message (service does not need setting up a relay)
        publisher = {'pub': 'apollo', 'sub': 'ros', 'service': ''}[type]
        
        if publisher:
            # Send message to bridge to setup relay
            msg = MSG_CONNECT.format(publisher=str(publisher), name=str(name))
            dest = '/topic/' + self._bridge_id + 'bridge_config'
            self._conn.send(body=str(msg), destination=str(dest))  # To config!
        else:
            # Ask for msg structure directly
            msg = MSG_INFO.format(type='srv-structure', name=str(name))
            dest = '/topic/' + self._bridge_id + 'bridge_info'
            self._conn.send(body=str(msg), destination=str(dest)) # To info!
    
    
    def on_error(self, headers, message):
        print('control received an error %s (%s)' % (message, headers))
    
    
    def on_message(self, headers, message):
        id = self._bridge_id
        if headers['destination'] == '/topic/' + id + 'bridge_status':
            root = et.fromstring(message)
            
            # Process errors
            for e in root.findall('error'):
                print('ERROR: ' + e.text)
            
            # Process relay information
            for r in root.findall('relay'):
                publisher = r.get('publisher')
                topic = r.get('name')
                status = '%s publisher to %s: %s' % (publisher, topic, r.text)
                print(status)
                # Is this what we wanted? If so, ask for the structure
                if r.text == 'ok' and topic == self._connect_request[1]:
                    msg = MSG_INFO.format(type='msg-structure', name=str(self._connect_request[1]))
                    self._conn.send(body=str(msg), destination='/topic/' + id + 'bridge_info') # To info!
        
        elif headers['destination'] == '/topic/' + id + 'bridge_info':
            root = et.fromstring(message)
            
            # Process answers
            for r in root.findall('response'):
                type = r.get('type')
                
                if type == 'msg-structure':
                    name = r.get('name')
                    answer = 'Received struct for %s' % name
                    # Is this what we wanted? If so, build the structure as a GUI
                    if name == self._connect_request[1]:
                        xml = et.tostring(r, 'utf-8')
                        self.structReceived.emit(self._connect_request, xml, '')
                
                elif type == 'srv-structure':
                    name = r.get('name')
                    # Is this what we wanted? If so, build the structure as a GUI
                    if name == self._connect_request[1]:
                        xml1 = et.tostring(r.find('request-structure'), 'utf-8')
                        xml2 = et.tostring(r.find('response-structure'), 'utf-8')
                        self.structReceived.emit(self._connect_request, xml1, xml2)
                
                elif type == 'list-pubs-subs-services':
                    # Collect pubs, subs, and services, 
                    pubs, subs, services = [], [], []
                    for item in r:
                        if item.tag == 'pub-topic':
                            pubs.append(item.text)
                        elif item.tag == 'sub-topic':
                            subs.append(item.text)
                        elif item.tag == 'service':
                            services.append(item.text)
                        else:
                            print(item.tag)
                    # Pass to topic select
                    # Note that what is pub for ROS, is sub for us!!
                    if hasattr(self, '_topicSelect'):
                        self._topicSelect.setPubsSubsServices(subs, pubs, services)
                else:
                    print(type)
        
        else:
            # Dispatch message to all known topic displays
            for td in self._topicDisplays:
                td.on_message(headers, message)

    
    def closeEvent(self, event):
        for td in self._topicDisplays:
            td.close()
        #self._conn.stop()
        super(GenericController, self).closeEvent(event)



class TopicSelect(QtGui.QWidget):
    
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent)
        self._controller = parent
        
        # Create field to set bridge name
        self._bridge_name_label = QtGui.QLabel('Bridge-id:', self)
        self._bridge_name = QtGui.QLineEdit(self._controller._bridge_id, self)
        self._bridge_name.setPlaceholderText('bridge id')
        self._bridge_name.setToolTip('Identifier of bridge to communicate with')
        self._bridge_name.editingFinished.connect(self.onBridgeNameChanged)
        # Dont like having focus, because we force an update if we leave focus
        self._bridge_name.setFocusPolicy(QtCore.Qt.ClickFocus) 
        
#         # Create refresh button
#         self._refresh_but = QtGui.QPushButton('Refresh', self)
#         self._refresh_but.clicked.connect(self.onRefresh)
        
        # Create dropdown box
        self._selection_label = QtGui.QLabel('Topic/service:', self)
        self._selection = QtGui.QComboBox(self)
        self._selection.setEditable(True)
        self._selection.setEditText('')
        #
        completer = self._selection.completer()
        completer.setCompletionMode(completer.PopupCompletion)
        
        # Connect button
        self._submit_but = QtGui.QPushButton('Create', self)
        self._submit_but.setEnabled(False)
        self._submit_but.clicked.connect(self.onSubmit)
        
        # Apply signals
        self._selection.currentIndexChanged.connect(self.onChanged)
        self._selection.editTextChanged.connect(self.onChanged)
        
        # layout
        layout = QtGui.QHBoxLayout(self)
        self.setLayout(layout)
        #
        layout.addWidget(self._bridge_name_label, 0)
        layout.addWidget(self._bridge_name, 1)
        layout.addWidget(self._selection_label, 0)
        layout.addWidget(self._selection, 3)
#         layout.addWidget(self._refresh_but, 0)
        layout.addWidget(self._submit_but, 0)
    
    
    def setPubsSubsServices(self, pubs, subs, services):
        # Make list of all items
        items = []
        items += ['pub:'+pub.lstrip('/') for pub in pubs]
        items += ['sub:'+sub.lstrip('/') for sub in subs]
        items += ['service:'+service.lstrip('/') for service in services]
        # Sort by name (not category)
        items.sort(key=lambda x:x.split(':')[1])
        # Add to selection combo box
        text = self._selection.currentText()
        self._selection.clear()
        for item in items:
            self._selection.addItem(item)
        self._selection.setEditText(text)
    
    
    def _getTypeAndName(self):
        # Get text and split name and type
        text = str(self._selection.currentText())
        type, _, name = text.partition(':')
        return type.strip(), name.strip()
    
    
    def onBridgeNameChanged(self):
        unstripped_bridge_id = str(self._bridge_name.text())
        bridge_id = unstripped_bridge_id.rstrip('_')
        if bridge_id.isalnum():
            bridge_id = bridge_id + '_'
            self._controller._bridge_id = bridge_id
            if bridge_id not in self._controller._subscribed_bridge_ids:
                self._controller._subscribed_bridge_ids.append(bridge_id)
                status = '/topic/' + bridge_id + 'bridge_status'
                info = '/topic/' + bridge_id + 'bridge_info'
                self._controller._conn.subscribe(destination=status, ack='auto', id='1')
                self._controller._conn.subscribe(destination=info, ack='auto', id='2')
            self.onRefresh()
        else:
            pass # bridge name will be reset below
        # Update text
        self._bridge_name.setText(self._controller._bridge_id.rstrip('_'))
    
    
    def onRefresh(self):
        # Clear first
        self.setPubsSubsServices([],[],[])
        # Send request for the lists
        dest = '/topic/' + self._controller._bridge_id + 'bridge_info'
        self._controller._conn.send(body=str(MSG_TOPIC_LIST), destination=str(dest))
    
    
    def onChanged(self):
        type, name = self._getTypeAndName()
        # Check if ok
        if type and name:
            self._submit_but.setEnabled(True)
        else:
            self._submit_but.setEnabled(False)
    
    
    def onSubmit(self):
        self._controller.makeConnection(*self._getTypeAndName())
        self._selection.setEditText('')



class TopicDockWidget(QtGui.QDockWidget):
    """ A dock widget that holds a topic display.
    """
        
    def __init__(self, parent):
        QtGui.QDockWidget.__init__(self, parent)
        
        # Set other settings
        self.setFeatures(   QtGui.QDockWidget.DockWidgetMovable |
                            QtGui.QDockWidget.DockWidgetClosable |
                            QtGui.QDockWidget.DockWidgetFloatable
                            #QtGui.QDockWidget.DockWidgetVerticalTitleBar
                            )

    def closeEvent(self, event):
        self.widget().close()
        self.parent()._topicDisplays.remove(self.widget())
        super(TopicDockWidget, self).closeEvent(event)

# Pick one!
def main():
    app = QtGui.QApplication(sys.argv)
    control = GenericController()
    control.show()
    control.resize(600,600)
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()

