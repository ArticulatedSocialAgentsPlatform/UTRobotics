
=================
Setting things up
=================

We use one code repository for the different components of the system.
This repository is a mercurial DVCS hosted at bitbucket.org.

The ROS modules are implemented in Python, as well as most of the
user interface applications. We use the "stomp.py" package for
communication with the apollo broker.


The Apollo broker
------------------

You can run the apollo broker on any PC, as long as its on the same
network. In principle the apollo broker is not run at the control 
computer that also runs ROS. In case of doubt, it's probably easiest
to run the broker at the same computer that runs the user interface
application.

To install Apollo:
  * Download the binary (google for "download apollo").
  * Execute ``bin/apollo``
  * Type ``create foo``, where "foo" is the name of your apollo server.

To use Apollo:
  * Start broker: ``foo/bin/appollo-broker``
  * Type ``run``
  * Your apollo server is now on!


ROS 
---

Note that the ROS operating system can also be run as a virtual machine.

To create the ROS OS:
  * Install Ubuntu
  * Follow instructions: http://www.ros.org/wiki/fuerte/Installation/Ubuntu

To create a ROS module:
  * Create a workspace with ``rosws init ~/ros_workspace /opt/ros/fuerte``
  * Create new package anywhere in this workspace ``roscreate-pkg package_name``

We use a single code repository, which we checkout inside our workspace.
The packages are simply directories in our code repository.

To use ROS:
  * Start broker in one shell: ``roscore``
  * In another shell first set the ROS PATH: ``. ros_workspace/setup.bash``
  * Now run whatever module you want with ``rosrun app script``



Installing the NAO modules on ROS
---------------------------------

See also the instructions here: http://www.ros.org/wiki/nao/Tutorials/Getting-Started

First we will need the rosdep package, which is not installed by
default:::
  
  sudo apt-get install python-rosdep
  sudo rosdep init
  rosdep update  (do not use sudo here!)

Then install the prerequisites:::
  
  sudo apt-get install ros-fuerte-joystick-drivers ros-fuerte-navigation ros-fuerte-rospack ros-fuerte-visualization
  rosdep install joy
  rosdep install navigation


Install the Nao packages:::
  
  rosdep install humanoid_msgs nao_robot nao_common
  rosmake humanoid_msgs nao_robot nao_common

Now install Naoqi:
  * Follow from `Installing NAOqi`: http://www.ros.org/wiki/nao/Tutorials/Getting-Started
  * (we had one occasion for which we needed to manually copy Python's include dir.)

.. Note::
  The robotplatform source code contains:
    * The Nao ROS module
    * A bash script to start the naoqi broker (official Nao)
    * A bash script to start the nao_driver (the ROS module)

To use Nao:
  * In a new shell run: ``robotframework/rosmodules/nao/naoqi_launch.sh``
  * In a new shell run: ``robotframework/rosmodules/nao/nao_launch.sh``

To turn visualization on:
  * The first time, follow instructions `Viewing the simulated Nao robot in rviz` in the above link
  * In a new shell run: ``roslaunch nao_description nao_state_publisher.launch``
  * In a new shell run: ``rosrun rviz rviz``
  
.. Note::
  If you are using the Nao with visualization there are thus at least
  six processes that run simultaneously on the ROS machine: 
  `ros broker`, `naoqi broker`, 
  `bridge module`, `nao module`, `nao state publisher`, `rviz`.



Motor fader interface
---------------------

For the motor fader, we've made use of the Behringer BCF2000. For the low
level MIDI communication, the Python package `rtmidi` is required. Other
packages may be suitable as well, but this was one of the few that worked,
is relatively easy to install, and is actively developed.

To install rtmidi on Windows:
  * A binary installer is provided that includes the rtmidi library

To install rtmidi on Linux:
  * Install libjack first (``sudo apt-get install libjack``)
  * Download the source from Pypi (search for 'python-rtmidi')
  * run ``sudo python setup.py install`` (or equivalent)

To install rtmidi on Mac:
  * Same as for Linux, except libjack is not required.


To use the motor fader ``from robotuils import bcfinterface``. Which
provides a simple API to control this particular motor fader via rtmidi.



User interface apps
-------------------

You need a valid Python system with stomp.py installed. We recommend 
Python 3.x with PySide or PyQt4.

