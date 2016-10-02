#!/usr/bin/env python3
# Copyright (C) 2013, Science Applied v.o.f.

""" Qt Widgets for displaying a message.
"""

import sys
import os
import time
import math
import xml.etree.ElementTree as et

import stomp

from robotutils.qt import QtGui, QtCore
from robotutils import xml2dict, dict2xml, get_config, set_config
from robotutils.xmlutils import Time

from valuedisplay import StructDisplay


class Display(QtGui.QScrollArea):
    """ Main display widget that shows one topic.
    Each display consists of a widget that shows the raw xml, as well
    as a widget that shows the message using structured GUI elements.
    """

    dataReceived = QtCore.Signal(str)

    def __init__(self, parent, conn, bridge_id):
        super(Display,self).__init__(parent)
        self._conn = conn
        self._bridge_id = bridge_id  # should already include the right prefix
        
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setWidgetResizable(True)
        self.setFrameShape(self.NoFrame)
        
        # Initialize state
        self._connect_request = '', ''
        self._apollo_topic = ''
        
        # Create splitter
        self._splitter = QtGui.QWidget(self)
        
        # Create three main widgets on the splitter
        self._sendDataCheck = QtGui.QCheckBox('Send data', self)
        self._sendDataCheck.stateChanged.connect(self.onCheckSendData)
        self._sendDataOnceCheck = QtGui.QCheckBox('Send data once', self)
        self._sendDataOnceCheck.stateChanged.connect(self.onCheckSendDataOnce)
        self._structVisibleCheck = QtGui.QCheckBox('Use raw xml', self)
        self._structVisibleCheck.stateChanged.connect(self.onCheck)
        self._structDisplay = QtGui.QWidget(self._splitter) #StructDisplay(self._splitter, struct)
        self._xmlDisplay = XmlDisplay(self._splitter)
        self._xmlDisplay.setPlainText('Connect to a topic ...')
        
        # Scroll on the spittter
        self.setWidget(self._splitter)
        
        self._lastTime = time.time()
        self.dataReceived.connect(self.pushXml)
        
        layout = QtGui.QVBoxLayout(self)
        self._splitter.setLayout(layout)
        layout.addWidget(self._sendDataCheck, 0)
        layout.addWidget(self._sendDataOnceCheck, 0)
        layout.addWidget(self._structVisibleCheck, 0)
        layout.addWidget(self._structDisplay, 0)
        layout.addWidget(self._xmlDisplay, 1)
        
        # Create timer to periodically send a control message (or the
        # turtle will stall)
        self._idleTimer = QtCore.QTimer()
        self._idleTimer.setInterval(500)
        self._idleTimer.setSingleShot(False)
        self._idleTimer.timeout.connect(self.sendMessage)
    
    
    def build(self, connect_request, xml1, xml2):
        """ Build the widgets to visualize the struct that is given via 
        the xml message.
        """
        self._connect_request = connect_request
        self._apollo_topic =  '/topic/' + self._bridge_id + self._connect_request[1].replace('/', '.')
        if connect_request[0] == 'service':
            self._apollo_topic = '/topic/' + self._bridge_id + 'bridge_service'
        
        # Create struct from XML
        struct = xml2dict(xml1)
        
        # Clear old one
        self._structDisplay.close()
        editable = isinstance(self, (PubDisplay, ServiceDisplay))
        self._structDisplay = StructDisplay(self._splitter, editable, struct)
        self._splitter.layout().insertWidget(1, self._structDisplay, 0)
        #self._splitter.addWidget(self._xmlDisplay) # Make sure its last
        
        # Update content with this dummy
        self.pushXml(xml1)
    
    
    def onCheck(self, state):
        visible = not state
        self._structDisplay.setVisible(visible)
        if visible:
            xml = str(self._xmlDisplay.toPlainText())
            self.pushXml(xml)

    def onCheckSendData(self, state):
        sending = not state
    
    def onCheckSendDataOnce(self, state):
        if (state!=False):
            self._sendDataOnceCheck.setCheckState(False)
            # Get xml, wrap it if necessary
            xml = str(self.pullXml())
            if self._connect_request[0] == 'service':
                xml = xml.replace("<data>", '<request name="%s">'%str(self._connect_request[1]))
                xml = xml.replace("</data>", '</request>')
                xml = "<service>%s</service>" % xml
            # send it away
            print('sending to topic \'%s\' \n message: \'%s\'' %(str(self._apollo_topic),str(xml)))
            self._conn.send(body=str(xml), destination=str(self._apollo_topic))
               
    
    def sendMessage(self):
        """ Pull message together and send it away.
        """
        if time.time() > self._lastTime + 0.01:
            self._lastTime = time.time()
            # Get xml, wrap it if necessary
            xml = str(self.pullXml())
            if self._connect_request[0] == 'service':
                xml = xml.replace("<data>", '<request name="%s">'%str(self._connect_request[1]))
                xml = xml.replace("</data>", '</request>')
                xml = "<service>%s</service>" % xml
            # send it away
            if self._sendDataCheck.checkState():
                print('sending to topic \'%s\' \n message: \'%s\'' %(str(self._apollo_topic),str(xml)))
                self._conn.send(body=str(xml), destination=str(self._apollo_topic))
    
    
    def on_message(self, headers, message):
        pass
    
    
    def onValueChanged(self):
        pass
    
    
    # todo: make this accept a dict instead?
    def pushXml(self, xml):
        """ Update the widgets to visualize the new data that is given via 
        the xml message.
        """
        struct = xml2dict(xml)
        xml = dict2xml(struct, pretty=True)  # Force pretty
        # Set xml 
        self._xmlDisplay.setPlainText(xml)
        # Push into the struct
        self._structDisplay.pushValue(struct)
    
    
    def pullXml(self):
        """ Pull the struct together in an xml string.
        """
        if not self._structVisibleCheck.checkState():
            # Pull the struct together
            d = self._structDisplay.pullValue()
            # Convert to xml
            xml = dict2xml(d, 'data', pretty=True)
            # Set xml and return
            self._xmlDisplay.setPlainText(xml)
        else:
            xml = str(self._xmlDisplay.toPlainText())
        return xml
    
    
    def closeEvent(self, event):
        super(Display, self).closeEvent(event)
        self._idleTimer.stop()
        print('closing topic display %s' % self._connect_request[1])



class PubDisplay(Display):
    """ Main display widget that shows one topic in publish mode.
    """
    def build(self, connect_request, xml1, xml2):
        Display.build(self, connect_request, xml1, xml2)
        # Start timer to periodically send data
        # Note that a message is also send when the sliders are used
        self._idleTimer.start()
    
    def onValueChanged(self):
        self.sendMessage()



class SubDisplay(Display):
    """ Main display widget that shows one topic in subscribe mode (readonly).
    """
    def build(self, connect_request, xml1, xml2):
        Display.build(self, connect_request, xml1, xml2)
        # Subscribe so we can continously update ourselves
        self._conn.subscribe(destination=self._apollo_topic, ack='auto', id='3')
    
    def on_message(self, headers, message):
        # This method gets called by the main GenericController widget
        if headers['destination'] == self._apollo_topic: 
            if time.time() > self._lastTime + 0.1:
                self._lastTime = time.time()
                self.dataReceived.emit(message)



class ServiceDisplay(Display):
    """ Main display widget that shows one service message. It consists
    of a request widget, a button to send the request, and a reply widget.
    """
    def __init__(self, *args, **kwargs):
        Display.__init__(self, *args, **kwargs)
        
        # Create button
        self._requestBut = QtGui.QPushButton("Do service request", self._splitter)
        self._requestBut.clicked.connect(self.sendMessage)
        
        # Create field to display response in
        self._xmlDisplay2 = XmlDisplay(self._splitter)
        
        # Add new widgets to splitter
        self._splitter.layout().addWidget(self._requestBut, 0)
        self._splitter.layout().addWidget(self._xmlDisplay2, 1)
    
    
    def build(self, connect_request, xml1, xml2):
        Display.build(self, connect_request, xml1, xml2)
        self._xmlDisplay2.setPlainText(xml2)
        # Subscribe so we can receive the response
        self._conn.subscribe(destination=self._apollo_topic, ack='auto', id='4')
    
    def on_message(self, headers, message):
        # This method gets called by the main GenericController widget
        if headers['destination'] == self._apollo_topic:
            root = et.fromstring(message)
            elems = root.findall('response')
            if elems:
                self.dataReceived.emit(message)
    
    def pushXml(self, xml):
        """ Update the widgets to visualize the new data that is given via 
        the xml message.
        """
        if '<response' in xml:
            self._xmlDisplay2.setPlainText(xml)
        else:
            super(ServiceDisplay, self).pushXml(xml)


    
class XmlDisplay(QtGui.QPlainTextEdit):
    """ Show XML code.
    """
    def __init__(self, parent):
        QtGui.QPlainTextEdit.__init__(self, parent)
        
        # Set, emit and return
        self.setFont(self.defaultFont())
    
    
    def defaultFont(cls):
        """ 
        Get the default (monospace) font for this system. Returns a QFont
        object. 
        Taken from IEP/codeeditor
        """
    
        # Get font size that makes sense for this system
        f = QtGui.QFont()
        size = f.pointSize()
        
        # Get font family 
        f = QtGui.QFont('lalala this is not a valid font name')
        f.setStyleHint(f.TypeWriter, f.PreferDefault)
        fi = QtGui.QFontInfo(f)
        family = fi.family()
        
        # Done
        return QtGui.QFont(family, size)
    
