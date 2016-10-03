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

from ram_output_msgs.msg import MidiOutput

global controllerOut

def midiOutputHandler(msg):
   # Handle message
   if msg.on:
     controllerOut.write_short(176, msg.button, 127)
   else:
     controllerOut.write_short(176, msg.button, 0)

def main():

   # Create ROS node for midi messages
   rospy.init_node('ram_midi_driver_output')
   rate = rospy.Rate(100)

   # Init mide controllers
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      rospy.logfatal("No MIDI devices detected")
      exit(-1)
   rospy.loginfo("Found %d MIDI devices", devices)

   # Get output device id
   if len(sys.argv) > 1:
      output_dev = int(sys.argv[1])
   else:
      output_dev = pygame.midi.get_default_output_id()
      if output_dev == -1:
         rospy.logfatal("No default MIDI output device")
         exit(-1)

   # Open controller
   rospy.loginfo("Using output device %d", output_dev)
   global controllerOut
   try:
     controllerOut = pygame.midi.Output(output_dev, 0)
   except pygame.midi.MidiException as e:
     # Retry, than it might work
     pygame.midi.quit()
     pygame.midi.init()
     devices = pygame.midi.get_count()
     rospy.loginfo("Second try: found %d MIDI devices", devices)
     controllerOut = pygame.midi.Output(output_dev, 0)

   # Reset buttons
   for i in range(0,8):
     controllerOut.write_short(176, 32 + i, 0)
     controllerOut.write_short(176, 48 + i, 0)
     controllerOut.write_short(176, 64 + i, 0)
   for i in range(0,6):
     controllerOut.write_short(176, 41 + i, 0)
   for i in range(0,5):
     controllerOut.write_short(176, 58 + i, 0)

   # Register handler
   rospy.Subscriber("/ram/output/midiout", MidiOutput, midiOutputHandler)

   # Main loop
   rospy.spin()

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
