import stomp

from PySide import QtGui, QtCore

LOGIN = {   'host_and_ports': [('192.168.87.231', 61613,)], 
            'user': 'admin', 
            'passcode': 'password',
        }
        


class MyListener(object):
    def on_error(self, headers, message):
        print('received an error %s' % message)

    def on_message(self, headers, message):
        print('received header %s' % headers)
        print('received a message %s' % message)


conn = stomp.Connection(**LOGIN)
conn.set_listener('', MyListener())

conn.start()
conn.connect()

conn.subscribe(destination='/topic/turtle1.pose', ack='auto')
