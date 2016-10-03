#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix

import roslib;
import rospy

import pygame
import pygame.midi

import sys
import os

from ram_animation_msgs.msg import EnabledModule
from ram_input_msgs.msg import Midi
from ram_output_msgs.msg import MidiOutput

def main():

   # Create ROS node for midi messages
   rospy.init_node('ram_midi_driver_input')
   rate = rospy.Rate(100)

   # Init mide controllers
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      rospy.logfatal("No MIDI devices detected")
      exit(-1)
   rospy.loginfo("Found %d MIDI devices", devices)

   # Get input device id
   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      input_dev = pygame.midi.get_default_input_id()
      if input_dev == -1:
         rospy.logfatal("No default MIDI input device")
         exit(-1)

   # Open controller
   rospy.loginfo("Using input device %d", input_dev)
   try:
     controller = pygame.midi.Input(input_dev, 100)
   except pygame.midi.MidiException as e:
     # Retry, than it might work
     pygame.midi.quit()
     pygame.midi.init()
     devices = pygame.midi.get_count()
     rospy.loginfo("Second try: found %d MIDI devices", devices)
     controller = pygame.midi.Input(input_dev, 100)

   # Create midi publisher
   pub = rospy.Publisher('/ram/input/midi', Midi, queue_size=100)
   m = Midi()

   # Create enabledmodule publisher
   modulePub = rospy.Publisher('/ram/animation/enabled/nodes', EnabledModule, queue_size=1, latch=True)
   module = EnabledModule()
   module.midi = True
   module.hmmm = False
   modulePub.publish(module)

   # Create midioutput publisher
   midiOutputPub = rospy.Publisher('/ram/output/midiout', MidiOutput, queue_size=1)
   midiOutput = MidiOutput()

   # Main loop
   while not rospy.is_shutdown():

     # Check for data
     if controller.poll():

       # Get data
       midi_event_list = controller.read(100) 
       messages = {}

       # Loop the data to find the most up to data value
       for midi_event in midi_event_list:
         messages[midi_event[0][1]] = midi_event[0][2];

       # Loop found messages
       for channel, value in messages.iteritems():

         # Init safe shutdown with stop button
         if channel == 42 and value == 127:
           midiOutput.button = 42
           midiOutput.on = True
           midiOutputPub.publish(midiOutput)
           os.system("rosrun ram_safe_shutdown ram_safe_shutdown_node")
           rospy.shutdown("Stopped for shutdown!")
           break

         if channel == 46 and value == 127:
           module.midi = not module.midi
           module.hmmm = not module.hmmm
           modulePub.publish(module)
           if module.hmmm:
             midiOutput.button = 46
             midiOutput.on = True
             midiOutputPub.publish(midiOutput)
           else:
             midiOutput.button = 46
             midiOutput.on = False
             midiOutputPub.publish(midiOutput)

         # Create and publish message
         m.chan = channel
         m.val  = value
         pub.publish(m)

     # Sleep ROS node
     rospy.sleep(0.01)

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
