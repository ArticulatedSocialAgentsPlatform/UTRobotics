; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude JointAnglesWithSpeedResult.msg.html

(cl:defclass <JointAnglesWithSpeedResult> (roslisp-msg-protocol:ros-message)
  ((goal_position
    :reader goal_position
    :initarg :goal_position
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass JointAnglesWithSpeedResult (<JointAnglesWithSpeedResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointAnglesWithSpeedResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointAnglesWithSpeedResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<JointAnglesWithSpeedResult> is deprecated: use nao_msgs-msg:JointAnglesWithSpeedResult instead.")))

(cl:ensure-generic-function 'goal_position-val :lambda-list '(m))
(cl:defmethod goal_position-val ((m <JointAnglesWithSpeedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:goal_position-val is deprecated.  Use nao_msgs-msg:goal_position instead.")
  (goal_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointAnglesWithSpeedResult>) ostream)
  "Serializes a message object of type '<JointAnglesWithSpeedResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointAnglesWithSpeedResult>) istream)
  "Deserializes a message object of type '<JointAnglesWithSpeedResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointAnglesWithSpeedResult>)))
  "Returns string type for a message object of type '<JointAnglesWithSpeedResult>"
  "nao_msgs/JointAnglesWithSpeedResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointAnglesWithSpeedResult)))
  "Returns string type for a message object of type 'JointAnglesWithSpeedResult"
  "nao_msgs/JointAnglesWithSpeedResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointAnglesWithSpeedResult>)))
  "Returns md5sum for a message object of type '<JointAnglesWithSpeedResult>"
  "1c77b3d9dc137611510fd16c3b792046")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointAnglesWithSpeedResult)))
  "Returns md5sum for a message object of type 'JointAnglesWithSpeedResult"
  "1c77b3d9dc137611510fd16c3b792046")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointAnglesWithSpeedResult>)))
  "Returns full string definition for message of type '<JointAnglesWithSpeedResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# result is the actually reached position~%sensor_msgs/JointState goal_position~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointAnglesWithSpeedResult)))
  "Returns full string definition for message of type 'JointAnglesWithSpeedResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# result is the actually reached position~%sensor_msgs/JointState goal_position~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointAnglesWithSpeedResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointAnglesWithSpeedResult>))
  "Converts a ROS message object to a list"
  (cl:list 'JointAnglesWithSpeedResult
    (cl:cons ':goal_position (goal_position msg))
))
