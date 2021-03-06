; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude JointAnglesWithSpeedAction.msg.html

(cl:defclass <JointAnglesWithSpeedAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type nao_msgs-msg:JointAnglesWithSpeedActionGoal
    :initform (cl:make-instance 'nao_msgs-msg:JointAnglesWithSpeedActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type nao_msgs-msg:JointAnglesWithSpeedActionResult
    :initform (cl:make-instance 'nao_msgs-msg:JointAnglesWithSpeedActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type nao_msgs-msg:JointAnglesWithSpeedActionFeedback
    :initform (cl:make-instance 'nao_msgs-msg:JointAnglesWithSpeedActionFeedback)))
)

(cl:defclass JointAnglesWithSpeedAction (<JointAnglesWithSpeedAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointAnglesWithSpeedAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointAnglesWithSpeedAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<JointAnglesWithSpeedAction> is deprecated: use nao_msgs-msg:JointAnglesWithSpeedAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <JointAnglesWithSpeedAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:action_goal-val is deprecated.  Use nao_msgs-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <JointAnglesWithSpeedAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:action_result-val is deprecated.  Use nao_msgs-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <JointAnglesWithSpeedAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:action_feedback-val is deprecated.  Use nao_msgs-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointAnglesWithSpeedAction>) ostream)
  "Serializes a message object of type '<JointAnglesWithSpeedAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointAnglesWithSpeedAction>) istream)
  "Deserializes a message object of type '<JointAnglesWithSpeedAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointAnglesWithSpeedAction>)))
  "Returns string type for a message object of type '<JointAnglesWithSpeedAction>"
  "nao_msgs/JointAnglesWithSpeedAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointAnglesWithSpeedAction)))
  "Returns string type for a message object of type 'JointAnglesWithSpeedAction"
  "nao_msgs/JointAnglesWithSpeedAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointAnglesWithSpeedAction>)))
  "Returns md5sum for a message object of type '<JointAnglesWithSpeedAction>"
  "efd2f7ac88847414fd26aacf32f993a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointAnglesWithSpeedAction)))
  "Returns md5sum for a message object of type 'JointAnglesWithSpeedAction"
  "efd2f7ac88847414fd26aacf32f993a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointAnglesWithSpeedAction>)))
  "Returns full string definition for message of type '<JointAnglesWithSpeedAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%JointAnglesWithSpeedActionGoal action_goal~%JointAnglesWithSpeedActionResult action_result~%JointAnglesWithSpeedActionFeedback action_feedback~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%JointAnglesWithSpeedGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal: a registered body pose name~%nao_msgs/JointAnglesWithSpeed joint_angles~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeed~%Header header~%~%# A list of joint names, corresponding to their names in the Nao docs.~%# This must be either the same lenght of joint_angles or 1 if it's a~%# keyword such as 'Body' (for all angles)~%string[] joint_names~%float32[] joint_angles~%~%#fraction of max joint velocity [0:1]~%float32 speed~%~%# Absolute angle(=0, default) or relative change~%uint8 relative~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%JointAnglesWithSpeedResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# result is the actually reached position~%sensor_msgs/JointState goal_position~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%JointAnglesWithSpeedFeedback feedback~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no feedback currently ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointAnglesWithSpeedAction)))
  "Returns full string definition for message of type 'JointAnglesWithSpeedAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%JointAnglesWithSpeedActionGoal action_goal~%JointAnglesWithSpeedActionResult action_result~%JointAnglesWithSpeedActionFeedback action_feedback~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%JointAnglesWithSpeedGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# goal: a registered body pose name~%nao_msgs/JointAnglesWithSpeed joint_angles~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeed~%Header header~%~%# A list of joint names, corresponding to their names in the Nao docs.~%# This must be either the same lenght of joint_angles or 1 if it's a~%# keyword such as 'Body' (for all angles)~%string[] joint_names~%float32[] joint_angles~%~%#fraction of max joint velocity [0:1]~%float32 speed~%~%# Absolute angle(=0, default) or relative change~%uint8 relative~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%JointAnglesWithSpeedResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# result is the actually reached position~%sensor_msgs/JointState goal_position~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%JointAnglesWithSpeedFeedback feedback~%~%================================================================================~%MSG: nao_msgs/JointAnglesWithSpeedFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no feedback currently ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointAnglesWithSpeedAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointAnglesWithSpeedAction>))
  "Converts a ROS message object to a list"
  (cl:list 'JointAnglesWithSpeedAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
