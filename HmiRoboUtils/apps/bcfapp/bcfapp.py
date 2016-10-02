
import sys
import math

from PyQt4 import QtGui, QtCore
import stomp
from stomp.listener import ConnectionListener

from robotutils import BCFInterface
from robotutils import get_config, set_config
from robotutils import xml2dict, dict2xml


LOGIN = get_config('apollo',    host_and_ports=[('localhost', 61613)])

MSG_CONNECT = """
<config>
 <relay publisher="ros" name="turtle1/pose" />
 <relay publisher="apollo" name="turtle1/command_velocity" />
</config>
"""

# Turtle pose limits
X_MIN = 0  # Horizontal position
X_MAX = 11
Y_MIN = 0  # Vertical position
Y_MAX = 11
THETA_MIN = 0  # Angular position
THETA_MAX = 2*math.pi

# Turtle command limits
V_MIN = -4  # Linear velocity
V_MAX = 4
W_MIN = -4  # Rotational velocity
W_MAX = 4

# Fader mapping
FADER_X = 1
FADER_Y = 2
FADER_THETA = 3
FADER_V = 7
FADER_W = 8

# Fader limits
FADER_MIN = 0
FADER_MAX = 127

def limit(x, min, max):
    if x > max:
        y = max
    elif x < min:
        y = min
    else:
        y = x
    
    return x


class FaderControl(QtGui.QWidget, ConnectionListener):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.setLayout(QtGui.QHBoxLayout())
        
        # Connect to BCF2000
        self._bcf = BCFInterface()
        self._bcf.fader_callback = self._set_virtual_fader
        
        
        # Create connection
        self._conn = stomp.Connection(**LOGIN)
        # Add listener
        self._conn.set_listener('fadercontrol', self)
        # Start the connection
        self._conn.start()
        self._conn.connect(username='admin',passcode='password')
        
        # Connect to Apollo (assuming that bridge is named 'test')
        self._conn.subscribe(
            destination='/topic/test_turtle1.pose', ack='auto', id=1)
        self._conn.subscribe(
            destination='/topic/test_turtle1.command_velocity', ack='auto', id=2)    
        
        # Add sliders to layout
        self._sliders = []
        for i in range(8):
            slider = QtGui.QSlider(QtCore.Qt.Vertical)
            slider.setRange(FADER_MIN, FADER_MAX)
            slider.setValue(0)
            slider.valueChanged.connect(lambda v, id=i+1:
                self._set_real_fader(id, v))
            
            self.layout().addWidget(slider)
            self._sliders.append(slider)
        
        # Turtle state
        self._state = {'x': 0, 'y': 0, 'theta': 0}
        self._control = {'linear': 0, 'angular': 0}
        self._map_control_to_fader()
        
        # Create timer to periodically send a control message
        self._timer = QtCore.QTimer()
        self._timer.setInterval(500)
        self._timer.setSingleShot(False)
        self._timer.timeout.connect(self._send_control)
        self._timer.start()
        
        # Connect relays (assuming that bridge is named 'test')
        self._conn.send(body=MSG_CONNECT, destination='/topic/test_bridge_config')
        
        # Show
        self.resize(400, 200)
        self.show()
    
    def on_message(self, headers, message):
        """ Callback for messages on the Apollo network. """
        
        destination = headers['destination']
        if destination == '/topic/test_turtle1.pose':
            # Map current pose onto sliders
            state = xml2dict(message)
            if state != self._state:
                self._state = state
                self._map_pose_to_fader()
        elif destination == '/topic/test_turtle1.command_velocity':
            # Control messages; ignore
            pass
    
    def set_fader(self, id, value):
        """ Set faders (real and virtual). """
        self._set_real_fader(id, value)
        self._set_virtual_fader(id, value)
    
    def _send_control(self):
        self._map_fader_to_control()
        msg = dict2xml(self._control)
        try:
            self._conn.send(body=msg,
                destination='/topic/test_turtle1.command_velocity')
        except Exception as e:
            print(e)
            self._timer.stop()
            raise
    
    def _map_control_to_fader(self):
        control = self._control
        
        # Fader value for linear velocity
        v = limit(control['linear'], V_MIN, V_MAX)
        v_scaling = (FADER_MAX - FADER_MIN)/(V_MAX - V_MIN)
        v_fader = int(v_scaling*v + (FADER_MAX - FADER_MIN)/2)
        
        # Fader value for angular velocity
        w = limit(control['angular'], W_MIN, W_MAX)
        w_scaling = (FADER_MAX - FADER_MIN)/(W_MAX - W_MIN)
        w_fader = int(w_scaling*w + (FADER_MAX - FADER_MIN)/2)
        
        # Write fader values
        self.set_fader(FADER_V, v_fader)
        self.set_fader(FADER_W, w_fader)
    
    def _map_fader_to_control(self):
        # Control value for linear velocity
        v_fader = self._bcf.get_fader(FADER_V)
        v_scaling = (FADER_MAX - FADER_MIN)/(V_MAX - V_MIN)
        v = (v_fader - (FADER_MAX - FADER_MIN)/2)/v_scaling
        self._control['linear'] = v
        
        # Control value for angular velocity
        w_fader = self._bcf.get_fader(FADER_W)
        w_scaling = (FADER_MAX - FADER_MIN)/(W_MAX - W_MIN)
        w = (w_fader - (FADER_MAX - FADER_MIN)/2)/w_scaling
        self._control['angular'] = w
        
    def _map_pose_to_fader(self):
        state = self._state
        
        # Fader value for x-position
        x = limit(state['x'], X_MIN, X_MAX)        
        x_scaling = (FADER_MAX - FADER_MIN)/(X_MAX - X_MIN)
        x_fader = int(x_scaling*x)
        
        # Fader value for y-position
        y = limit(state['y'], Y_MIN, Y_MAX)
        y_scaling = (FADER_MAX - FADER_MIN)/(Y_MAX - Y_MIN)
        y_fader = int(y_scaling*y)
        
        # Fader value for orientation
        theta = state['theta'] % (2*math.pi)
        theta_scaling = (FADER_MAX - FADER_MIN)/(THETA_MAX - THETA_MIN)
        theta_fader = int(theta_scaling*theta)
        
        # Write fader values
        self.set_fader(FADER_X, x_fader)
        self.set_fader(FADER_Y, y_fader)
        self.set_fader(FADER_THETA, theta_fader)
    
    def _set_real_fader(self, id, value):
        # If virtual fader is moved: move real fader accordingly
        if self._bcf.get_fader(id) != value:
            self._bcf.set_fader(id, value)
        
        self._send_control()
    
    def _set_virtual_fader(self, id, value):
        # If real fader is moved: move virtual fader accordingly
        if self._sliders[id-1].value() != value:
            self._sliders[id-1].setValue(value)
        
        self._send_control()

    
    
# Pick one!
def main():
    app = QtGui.QApplication(sys.argv)
    control = FaderControl()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()