; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude JointTrajectoryFeedback.msg.html

(cl:defclass <JointTrajectoryFeedback> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JointTrajectoryFeedback (<JointTrajectoryFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointTrajectoryFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointTrajectoryFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<JointTrajectoryFeedback> is deprecated: use nao_msgs-msg:JointTrajectoryFeedback instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointTrajectoryFeedback>) ostream)
  "Serializes a message object of type '<JointTrajectoryFeedback>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointTrajectoryFeedback>) istream)
  "Deserializes a message object of type '<JointTrajectoryFeedback>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointTrajectoryFeedback>)))
  "Returns string type for a message object of type '<JointTrajectoryFeedback>"
  "nao_msgs/JointTrajectoryFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointTrajectoryFeedback)))
  "Returns string type for a message object of type 'JointTrajectoryFeedback"
  "nao_msgs/JointTrajectoryFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointTrajectoryFeedback>)))
  "Returns md5sum for a message object of type '<JointTrajectoryFeedback>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointTrajectoryFeedback)))
  "Returns md5sum for a message object of type 'JointTrajectoryFeedback"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointTrajectoryFeedback>)))
  "Returns full string definition for message of type '<JointTrajectoryFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no feedback currently ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointTrajectoryFeedback)))
  "Returns full string definition for message of type 'JointTrajectoryFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no feedback currently ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointTrajectoryFeedback>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointTrajectoryFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'JointTrajectoryFeedback
))
