; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude BodyPoseResult.msg.html

(cl:defclass <BodyPoseResult> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BodyPoseResult (<BodyPoseResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyPoseResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyPoseResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<BodyPoseResult> is deprecated: use nao_msgs-msg:BodyPoseResult instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyPoseResult>) ostream)
  "Serializes a message object of type '<BodyPoseResult>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyPoseResult>) istream)
  "Deserializes a message object of type '<BodyPoseResult>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyPoseResult>)))
  "Returns string type for a message object of type '<BodyPoseResult>"
  "nao_msgs/BodyPoseResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyPoseResult)))
  "Returns string type for a message object of type 'BodyPoseResult"
  "nao_msgs/BodyPoseResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyPoseResult>)))
  "Returns md5sum for a message object of type '<BodyPoseResult>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyPoseResult)))
  "Returns md5sum for a message object of type 'BodyPoseResult"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyPoseResult>)))
  "Returns full string definition for message of type '<BodyPoseResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no result currently~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyPoseResult)))
  "Returns full string definition for message of type 'BodyPoseResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# no result currently~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyPoseResult>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyPoseResult>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyPoseResult
))
