

Rob = '192.168.87.231', 'rob', 'rob'

import time
import sys

import stomp

from PySide import QtGui, QtCore

LOGIN = {   'host_and_ports': [('localhost', 61613,)], 
            'user': 'admin', 
            'passcode': 'password',
        }


class Listener(object):
    def on_error(self, headers, message):
        print('received an error %s' % message)

    def on_message(self, headers, message):
        print('received a message %s' % message)



class MainWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent)
        
        
    def _layout(self):
        pass
    
    def _connect(self):
        
        # Create connection
        self._conn = stomp.Connection(**login)
        # Add listener
        listener = Listener()
        self._conn.set_listener('', listener)
        # Set subscriptions
        conn.subscribe(destination='/queue/test', ack='auto')
        # Start the connection
        self._conn.start()
        self._conn.connect()
    
    
    def closeEvent(self, event):
        print('closing connection')
        self._conn.close()
        QtGui.QWidget.closeEvent(self, event)



#conn.send(' '.join(sys.argv[1:]), destination='/queue/test')

#conn.disconnect()
