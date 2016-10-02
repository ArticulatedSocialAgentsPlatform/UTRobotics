------------
Applications
------------

The ``apps`` directory contains the front-end applications for the robot
platform. These can be user interfaces, but can for instance also be
programs that control a robot in other ways.

Below is a list of applciations. Most do not exist yet, but give an
idea about the possibilities of the platform.


Turtle app
----------

The turtle application is a simple application to control the turtlesim
robot. This is a standard simulation robot (software only). It is mainly
intended for demonstartion and test purposes.

The application allows the user to control the speed and change if direction
of a virtual turtle. The turtle robot sends back its coordinates, allowing
the application to draw the turtle in the correct place.

There is also a turtlesim application, which acts like the turle robot, 
but communicates directly via the appollo network. This can be used for
testing purposes when a ROS backend is not available.


Generic control app
-------------------

.. automodule:: genericcontrolapp


NAO app
-------

For controlling the NAO. (Not implemented, use the generic control app instead.)


Motorfader app
--------------

App that interfaces the motor fader with the apollo network. We provide
an example to control the turtlesim module, which can be modified to control
other ROS modules as well.


Blender app
-----------

A Python module to allow blender to be used as a front-end for robot control.
(not implemented)

