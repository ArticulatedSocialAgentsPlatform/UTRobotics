; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude BlinkFeedback.msg.html

(cl:defclass <BlinkFeedback> (roslisp-msg-protocol:ros-message)
  ((last_color
    :reader last_color
    :initarg :last_color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA)))
)

(cl:defclass BlinkFeedback (<BlinkFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlinkFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlinkFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<BlinkFeedback> is deprecated: use nao_msgs-msg:BlinkFeedback instead.")))

(cl:ensure-generic-function 'last_color-val :lambda-list '(m))
(cl:defmethod last_color-val ((m <BlinkFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:last_color-val is deprecated.  Use nao_msgs-msg:last_color instead.")
  (last_color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlinkFeedback>) ostream)
  "Serializes a message object of type '<BlinkFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'last_color) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlinkFeedback>) istream)
  "Deserializes a message object of type '<BlinkFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'last_color) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlinkFeedback>)))
  "Returns string type for a message object of type '<BlinkFeedback>"
  "nao_msgs/BlinkFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlinkFeedback)))
  "Returns string type for a message object of type 'BlinkFeedback"
  "nao_msgs/BlinkFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlinkFeedback>)))
  "Returns md5sum for a message object of type '<BlinkFeedback>"
  "6f1f94fb3eb06412264f6e0c5e72cfab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlinkFeedback)))
  "Returns md5sum for a message object of type 'BlinkFeedback"
  "6f1f94fb3eb06412264f6e0c5e72cfab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlinkFeedback>)))
  "Returns full string definition for message of type '<BlinkFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%std_msgs/ColorRGBA last_color~%~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlinkFeedback)))
  "Returns full string definition for message of type 'BlinkFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%std_msgs/ColorRGBA last_color~%~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlinkFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'last_color))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlinkFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'BlinkFeedback
    (cl:cons ':last_color (last_color msg))
))
