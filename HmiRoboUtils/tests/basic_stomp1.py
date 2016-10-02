""" Example illustrating basic stomp based on stomp.py
"""

import stomp

## Common for both sides

login = {'host_and_ports': [('localhost', 61613,)], 
        'user': 'admin', 
        'passcode': 'password',
        }
    

conn = stomp.Connection(**login)
conn.start()  # According to docs, must be called after setting listeners
conn.connect()



## Subscribing side

class MyListener(object):
    def on_error(self, headers, message):
        print('received an error %s' % message)

    def on_message(self, headers, message):
        print('received header %s' % headers)
        print('received a message %s' % message)


conn.set_listener('', MyListener())


conn.subscribe(destination='/topic/turtle1.control', ack='auto')
conn.subscribe(destination='/topic/turtle1.pose', ack='auto')


## Producer side

#conn.start() 
conn.send('lala1', destination='/topic/turtle1.control')
conn.send('lala2', destination='/topic/turtle1.pose')

# conn.disconnect()  # Disconnects
# conn.stop()  # Like disconnect but with nice cleanup and wait for conn-thread
