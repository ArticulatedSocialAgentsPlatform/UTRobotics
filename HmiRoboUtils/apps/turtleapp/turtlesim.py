""" 
Script to run the turtle app using only apollo.
This kind of does what the turtlesim_node does in ROS.

This module is now deprecated and probably does not work.

"""

import time
import sys

import stomp

from robotutils.config import get_config, set_config

# Get login
LOGIN = get_config('apollo',    host_and_ports=[('localhost', 61613)])


MSG_CONTROL = """
<data>
    <linear>{linear}</linear>
    <angular>{angular}</angular>
</data>
"""

MSG_STATE = """
<data>
  <x type="float">{x}</x>
  <y type="float">{y}</y>
  <theta type="float">{theta}</theta>
  <linear_velocity type="float">{linear_velocity}</linear_velocity>
  <angular_velocity type="float">{angular_velocity}</angular_velocity>
</data>
"""

import xml.dom.minidom as dom

def xml_to_pydict(s):
    return _xml_to_pydict( dom.parseString(s).documentElement)

def _xml_to_pydict(element):
    ret = {}
    for child in element.childNodes:
        if child.nodeType != child.ELEMENT_NODE:
            continue
        tag = child.tagName
        if any([grandchild.nodeType == child.ELEMENT_NODE for grandchild in child.childNodes]):
            ret[tag] = _xml_to_pydict(child)
        else:
            type = eval  # todo: determine real type!
            ret[tag] = type(child.childNodes[0].data)
    return ret         


class Turtle:
    """ Simple class that represents the turle.
    It listens to control events. In the mean time it continously
    updates the state of the turle (x, y, theta) and sends state update
    messages.
    """ 
    
    def __init__(self, conn=None):
        print('Starting turtle')
        
        self._conn = conn
        if conn is None:
            self._connect()
        self._conn.set_listener('turtle', self)
        self._conn.subscribe(destination='/topic/test_turtle1.command_velocity', id=5, ack='auto')
        
        # Turtle state
        self._control = {}
        self._state = {'x': 0, 'y':0, 'theta':0, 
            'linear_velocity':0, 'angular_velocity':0}
        
        self.run()
        
    def run(self):
        import time
        import math
        delta = 0.5
        
        while True:
            time.sleep(delta)
            if not self._control:
                continue
            
            # Copy linear and angular velocity
            self._state['linear_velocity'] = self._control['linear']
            self._state['angular_velocity'] = self._control['angular']
            # Set new theta
            self._state['theta'] += self._control['angular']
            # Calcuate new state base ond the control
            d = self._control['linear'] * delta
            dx = d * math.cos(self._state['theta'])
            dy = d * math.sin(self._state['theta'])
            #
            self._state['x'] += dx
            self._state['y'] += dy
            
            statusMsg = ('x: '+'{0: <16}'.format(self._state['x'])+
                'y: '+'{0: <16}'.format(self._state['y'])+
                'th: '+'{0: <16}'.format(self._state['theta'])+
                'lv: '+'{0: <16}'.format(self._state['linear_velocity'])+
                'av: '+'{0: <16}'.format(self._state['angular_velocity']))
            print(statusMsg)
            
            # Update!
            self._sendStatusMessage()
    
    def _connect(self):
        self._conn = stomp.Connection(**LOGIN)
        self._conn.start()
        self._conn.connect(username='admin', passcode='password')
        
    
    def on_error(self, headers, message):
        print('turtle received an error %s' % message)

    def on_message(self, headers, message):
        #print('Got message: \'%s\' with headers: \'%s\'' %(message, headers))
        if headers['destination'] != '/topic/test_turtle1.command_velocity':
            return
        
        control = xml_to_pydict(message)
        if 'linear' in control:
            self._control['linear'] = control['linear']
        if 'angular' in control:
            self._control['angular'] = control['angular']

    
    def _sendStatusMessage(self):
        msg = MSG_STATE.format(**self._state)
        self._conn.send(body=msg, destination='/topic/test_turtle1.pose')


if __name__ == '__main__':
    turle = Turtle() 
