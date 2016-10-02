----------
Rosmodules
----------

The ``rosmodules`` directory contains the different ROS modules. Each 
module is again located in a separate directory. Each ROS modules
has a specific layout and meta dat in order to be recognized as 
a ROS module. The code for the ROS modules is located in the ``scripts``
directory within a ROS module.


Module: bridge
--------------

The bridge module contains one application: ``relay.py``. It handles
the conversion of ROS messages to STOMP messages and vice versa.

.. autoclass:: relay.Bridge
    :members:

Module: nao
-----------

The nao module is responsible for the control of the NAO robot. This
is an off-the-shelf robot: http://www.aldebaran-robotics.com/en/
