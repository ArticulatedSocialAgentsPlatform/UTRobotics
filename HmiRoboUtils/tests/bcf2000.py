import rtmidi

midiin = rtmidi.MidiIn()
p = midiin.open_port(1)

while True:
    m = midiin.get_message()
    if m:
        print(m)


##
import rtmidi

midiout = rtmidi.MidiOut()

for i in range(midiout.get_port_count()):
    print(i, midiout.get_port_name(i))

p = midiout.open_port(0)

import time
for i in range(255):
    time.sleep(0.5)
    print(i)
    midiout.send_message([i, 88, 100])
