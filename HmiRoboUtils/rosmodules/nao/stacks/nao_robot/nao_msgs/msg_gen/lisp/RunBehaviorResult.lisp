; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude RunBehaviorResult.msg.html

(cl:defclass <RunBehaviorResult> (roslisp-msg-protocol:ros-message)
  ((noErrors
    :reader noErrors
    :initarg :noErrors
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RunBehaviorResult (<RunBehaviorResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunBehaviorResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunBehaviorResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<RunBehaviorResult> is deprecated: use nao_msgs-msg:RunBehaviorResult instead.")))

(cl:ensure-generic-function 'noErrors-val :lambda-list '(m))
(cl:defmethod noErrors-val ((m <RunBehaviorResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:noErrors-val is deprecated.  Use nao_msgs-msg:noErrors instead.")
  (noErrors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunBehaviorResult>) ostream)
  "Serializes a message object of type '<RunBehaviorResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'noErrors) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunBehaviorResult>) istream)
  "Deserializes a message object of type '<RunBehaviorResult>"
    (cl:setf (cl:slot-value msg 'noErrors) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunBehaviorResult>)))
  "Returns string type for a message object of type '<RunBehaviorResult>"
  "nao_msgs/RunBehaviorResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunBehaviorResult)))
  "Returns string type for a message object of type 'RunBehaviorResult"
  "nao_msgs/RunBehaviorResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunBehaviorResult>)))
  "Returns md5sum for a message object of type '<RunBehaviorResult>"
  "e6b2d7551eb5c957b430b0edca37c8db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunBehaviorResult)))
  "Returns md5sum for a message object of type 'RunBehaviorResult"
  "e6b2d7551eb5c957b430b0edca37c8db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunBehaviorResult>)))
  "Returns full string definition for message of type '<RunBehaviorResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool noErrors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunBehaviorResult)))
  "Returns full string definition for message of type 'RunBehaviorResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool noErrors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunBehaviorResult>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunBehaviorResult>))
  "Converts a ROS message object to a list"
  (cl:list 'RunBehaviorResult
    (cl:cons ':noErrors (noErrors msg))
))