""" 
Small Qt app to control the turtlebot via Apollo and ROS.
"""


import sys
import time
import xml.etree.ElementTree as et

import stomp
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import qDebug

from robotutils import get_config, set_config
from robotutils import xml2dict, dict2xml
from robotutils.xmlutils import indent


# Get login
LOGIN = get_config('apollo',    host_and_ports=[('localhost', 61613)])


# Messages
MSG_CONNECT = """
<config>
 <relay publisher="ros" name="turtle1/pose" />
 <relay publisher="apollo" name="turtle1/command_velocity" />
</config>
"""

MSG_INFO = """
<info>
 <request type="list-pubs-subs-services" />
 <request type="msg-structure" name="turtle1/pose" />
 <request type="msg-structure" name="turtle1/command_velocity" />
 <request type="srv-structure" name="spawn" />
 <request type="srv-structure" name="turtle1/teleport_absolute" />
</info>
"""


MSG_TELEPORT = """
<service>
 <request name="turtle1/teleport_absolute">
  <x type="float">0.0</x>
  <y type="float">0.0</y>
  <theta type="float">0.5</theta>
 </request>
</service>
"""


class Canvas(QtGui.QFrame):
    """ Canvas to draw the turtle in.
    """
    
    def __init__(self, parent=None):
        super(Canvas, self).__init__(parent)
        
        self.setMinimumHeight(100)
        self.setMinimumWidth(100)
        
        # Style the border of the frame
        self.setFrameShape(self.Box)
        self.setFrameShadow(self.Plain)
        self.setLineWidth(1)
        
        self._x, self._y, self._theta = 0, 0, 0
        self._world = 0, 11, 0, 11
    
    def setTurtle(self, x=0, y=0, theta=0, **kwargs):
        print('test')
        self._x, self._y, self._theta = float(x), float(y), float(theta)
        self.update()
    
    def paintEvent(self, event):
        super(Canvas, self).paintEvent(event)
        
        import math
        
        # Prepare
        xy = self._locToPixel(self._x, self._y)
        d = 3
        xy2 = xy[0] + 2.5*d*math.cos(self._theta), xy[1] - 2.5*d*math.sin(self._theta)
        
        
        # Create painter
        painter = QtGui.QPainter()
        painter.begin(self)
        
        pen1 = QtGui.QPen(QtGui.QColor(100,100,200))
        pen1.setWidth(8)
        pen2 = QtGui.QPen(QtGui.QColor(0,0,100))
        pen2.setWidth(2)
        
        painter.setRenderHint(painter.Antialiasing)
        # Draw body
        painter.setPen(pen1)
      #  xy = self._locToPixel(self._x, self._y)
        painter.drawPoint(*xy)
      #  d = 3
        painter.drawEllipse(xy[0]-d, xy[1]-d, 2*d, 2*d)
        # Draw head
        painter.setPen(pen2)
      #  xy2 = xy[0] + 2.5*d*math.cos(self._theta), xy[1] - 2.5*d*math.sin(self._theta)
        painter.drawLine(xy[0], xy[1], xy2[0], xy2[1])
        #
        painter.end()
        
        # Draw border
        QtGui.QFrame.paintEvent(self, event)
    
    def _locToPixel(self, x, y):
        pw, ph = self.width(), self.height()
        w = self._world[1] - self._world[0]
        h = self._world[3] - self._world[2]
        #
        px =       pw * ( x-self._world[0]) / w
        py =  ph - ph * ( y-self._world[2]) / h
        return px, py



class TurtleControl(QtGui.QWidget):
    """ Control widget for the turtle, and display the turtle state.
    """
    
    statusMsg = QtCore.pyqtSignal(str)
    
    def __init__(self, parent=None):
        super(TurtleControl, self).__init__(parent)
        print('Starting turtle control')
        
        # Status field
        self._status = QtGui.QPlainTextEdit(self)
        self._status.setReadOnly(True)
        
        # Create buttons
        self._infoBut = QtGui.QPushButton('Get message info', self)
        self._relayBut = QtGui.QPushButton('Connect relays', self)
        self._teleportBut = QtGui.QPushButton('Reset to origin', self)
        self._forwardBut = QtGui.QPushButton('Forward', self)
        self._backwardBut = QtGui.QPushButton('Backward', self)
        self._leftBut = QtGui.QPushButton('Left', self)
        self._rightBut = QtGui.QPushButton('Right', self)
        
        # Create canvas
        self._canvas = Canvas(self)
        
        # Turtle control state
        self._control = {'linear': 0, 'angular': 0}
        
        # Create signals
        self._infoBut.clicked.connect(self._sendInfoMessage)
        self._relayBut.clicked.connect(self._sendRelayMessage)
        self._teleportBut.clicked.connect(self._sendTeleportMessage)
        self._forwardBut.pressed.connect(self._onForwardDown)
#         self._forwardBut.released.connect(self._onForwardUp)
        self._backwardBut.pressed.connect(self._onBackwardDown)
#         self._backwardBut.released.connect(self._onBackwardUp)
        self._leftBut.pressed.connect(self._onLeftDown)
        self._rightBut.pressed.connect(self._onRightDown)
        self.statusMsg.connect(self._status.appendPlainText)
        
        
        # Init
        self._layout()
        self.show()
        self._connect()
        
        
        # Create timer to periodically send a control message (or the
        # turtle will stall)
        self._idleTimer = QtCore.QTimer()
        self._idleTimer.setInterval(500)
        self._idleTimer.setSingleShot(False)
        self._idleTimer.timeout.connect(self._sendControlMessage)
        self._idleTimer.start()
        
        self._sendInfoMessage()
    
    def _layout(self):
        # Create layout
        layout = QtGui.QVBoxLayout(self)
        self.setLayout(layout)
        # Add Forward
        sublayout = QtGui.QHBoxLayout()
        sublayout.addStretch(2)
        sublayout.addWidget(self._forwardBut, 2)
        sublayout.addStretch(2)
        layout.addLayout(sublayout)
        #
        sublayout = QtGui.QHBoxLayout()
        sublayout.addStretch(2)
        sublayout.addWidget(self._leftBut, 1)
        sublayout.addWidget(self._rightBut, 1)
        sublayout.addStretch(2)
        layout.addLayout(sublayout)
        #
        sublayout = QtGui.QHBoxLayout()
        sublayout.addStretch(2)
        sublayout.addWidget(self._backwardBut, 2)
        sublayout.addStretch(2)
        layout.addLayout(sublayout)
        
        layout.addWidget(self._teleportBut)
        layout.addWidget(self._canvas, 1)
        layout.addWidget(self._infoBut)
        layout.addWidget(self._relayBut)
        layout.addWidget(self._status, 1)
    
    def _connect(self):
        # Create connection
        self._conn = stomp.Connection(**LOGIN)
        # Add listener
        self._conn.set_listener('control', self)
        # Start the connection
        self._conn.start()
        self._conn.connect(username='admin',passcode='password')
        # Set subscriptions
        self._conn.subscribe(destination='/topic/test_bridge_status', id=1, ack='auto')
        self._conn.subscribe(destination='/topic/test_bridge_info', id=2, ack='auto')
        self._conn.subscribe(destination='/topic/test_bridge_service', id=3, ack='auto')
        self._conn.subscribe(destination='/topic/test_turtle1.pose', id=4, ack='auto')
        self._conn.subscribe(destination='/topic/test_turtle1.command_velocity', id=5, ack='auto')
    
    def _sendInfoMessage(self):
        self.statusMsg.emit('requesting info...')
        self._conn.send(body=MSG_INFO, destination='/topic/test_bridge_info')
    
    def _sendRelayMessage(self):
        self.statusMsg.emit('connecting relays...')
        self._conn.send(body=MSG_CONNECT, destination='/topic/test_bridge_config')
    
    def _sendTeleportMessage(self):
        self.statusMsg.emit('sending teleport service call...')
        self._conn.send(body=MSG_TELEPORT, destination='/topic/test_bridge_service')
    
    def _sendControlMessage(self):
        msg = dict2xml(self._control)
        try:
            self._conn.send(body=msg, destination='/topic/test_turtle1.command_velocity')
        except Exception:
            self._idleTimer.stop()
            raise
    
    def _onForwardDown(self):
        self._control['linear'] += 0.25
        self._sendControlMessage()

    def _onBackwardDown(self):
        self._control['linear'] -= 0.25
        self._sendControlMessage()
    
    def _onLeftDown(self):
        self._control['angular'] += 0.2  # rad/sec 
        self._sendControlMessage()
    
    def _onRightDown(self):
        self._control['angular'] -= 0.2  # rad/sec 
        self._sendControlMessage()
    
    def _onForwardUp(self):
        pass
    
    def _onBackwardUp(self):
        pass
    
    def on_error(self, headers, message):
        print('control received an error: \'%s\'' % headers)

    def on_message(self, headers, message):
        #print('Got message: \'%s\' with headers: \'%s\'' %(message, headers))
        destination = headers['destination']
        if destination == '/topic/test_turtle1.pose': 
            state = xml2dict(message)
            self._canvas.setTurtle(**state)
        elif destination == '/topic/test_turtle1.command_velocity':
            self._control = xml2dict(message)
        else:
            e = et.fromstring(message)
            indent(e)
            self.statusMsg.emit(et.tostring(e, 'utf-8'))
    
    def closeEvent(self, event):
        print('control closing connection')
        #self._conn.stop()
        self._idleTimer.stop()
        QtGui.QWidget.closeEvent(self, event)
        QtGui.QApplication.exit()


# Pick one!
def main():
    app = QtGui.QApplication(sys.argv)
    control = TurtleControl()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()
