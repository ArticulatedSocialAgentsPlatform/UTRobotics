; Auto-generated. Do not edit!


(cl:in-package nao_msgs-msg)


;//! \htmlinclude TorsoIMU.msg.html

(cl:defclass <TorsoIMU> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angleX
    :reader angleX
    :initarg :angleX
    :type cl:float
    :initform 0.0)
   (angleY
    :reader angleY
    :initarg :angleY
    :type cl:float
    :initform 0.0)
   (gyroX
    :reader gyroX
    :initarg :gyroX
    :type cl:float
    :initform 0.0)
   (gyroY
    :reader gyroY
    :initarg :gyroY
    :type cl:float
    :initform 0.0)
   (accelX
    :reader accelX
    :initarg :accelX
    :type cl:float
    :initform 0.0)
   (accelY
    :reader accelY
    :initarg :accelY
    :type cl:float
    :initform 0.0)
   (accelZ
    :reader accelZ
    :initarg :accelZ
    :type cl:float
    :initform 0.0))
)

(cl:defclass TorsoIMU (<TorsoIMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorsoIMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorsoIMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nao_msgs-msg:<TorsoIMU> is deprecated: use nao_msgs-msg:TorsoIMU instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:header-val is deprecated.  Use nao_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'angleX-val :lambda-list '(m))
(cl:defmethod angleX-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:angleX-val is deprecated.  Use nao_msgs-msg:angleX instead.")
  (angleX m))

(cl:ensure-generic-function 'angleY-val :lambda-list '(m))
(cl:defmethod angleY-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:angleY-val is deprecated.  Use nao_msgs-msg:angleY instead.")
  (angleY m))

(cl:ensure-generic-function 'gyroX-val :lambda-list '(m))
(cl:defmethod gyroX-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:gyroX-val is deprecated.  Use nao_msgs-msg:gyroX instead.")
  (gyroX m))

(cl:ensure-generic-function 'gyroY-val :lambda-list '(m))
(cl:defmethod gyroY-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:gyroY-val is deprecated.  Use nao_msgs-msg:gyroY instead.")
  (gyroY m))

(cl:ensure-generic-function 'accelX-val :lambda-list '(m))
(cl:defmethod accelX-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:accelX-val is deprecated.  Use nao_msgs-msg:accelX instead.")
  (accelX m))

(cl:ensure-generic-function 'accelY-val :lambda-list '(m))
(cl:defmethod accelY-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:accelY-val is deprecated.  Use nao_msgs-msg:accelY instead.")
  (accelY m))

(cl:ensure-generic-function 'accelZ-val :lambda-list '(m))
(cl:defmethod accelZ-val ((m <TorsoIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nao_msgs-msg:accelZ-val is deprecated.  Use nao_msgs-msg:accelZ instead.")
  (accelZ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorsoIMU>) ostream)
  "Serializes a message object of type '<TorsoIMU>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angleX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angleY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gyroX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gyroY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accelX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accelY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accelZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorsoIMU>) istream)
  "Deserializes a message object of type '<TorsoIMU>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyroX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyroY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accelX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accelY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accelZ) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorsoIMU>)))
  "Returns string type for a message object of type '<TorsoIMU>"
  "nao_msgs/TorsoIMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorsoIMU)))
  "Returns string type for a message object of type 'TorsoIMU"
  "nao_msgs/TorsoIMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorsoIMU>)))
  "Returns md5sum for a message object of type '<TorsoIMU>"
  "404112cb51a476613d7a445b4586a894")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorsoIMU)))
  "Returns md5sum for a message object of type 'TorsoIMU"
  "404112cb51a476613d7a445b4586a894")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorsoIMU>)))
  "Returns full string definition for message of type '<TorsoIMU>"
  (cl:format cl:nil "# Data from Nao's IMU, raw and filtered~%~%Header header~%~%# corrected / filtered angle to X axis (roll)~%float32 angleX~%~%# corrected / filtered angle to Y axis (pitch)~%float32 angleY~%~%# Raw data from gyroscopes~%float32 gyroX~%float32 gyroY~%~%# Raw data from accelerometers~%float32 accelX~%float32 accelY~%float32 accelZ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorsoIMU)))
  "Returns full string definition for message of type 'TorsoIMU"
  (cl:format cl:nil "# Data from Nao's IMU, raw and filtered~%~%Header header~%~%# corrected / filtered angle to X axis (roll)~%float32 angleX~%~%# corrected / filtered angle to Y axis (pitch)~%float32 angleY~%~%# Raw data from gyroscopes~%float32 gyroX~%float32 gyroY~%~%# Raw data from accelerometers~%float32 accelX~%float32 accelY~%float32 accelZ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorsoIMU>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorsoIMU>))
  "Converts a ROS message object to a list"
  (cl:list 'TorsoIMU
    (cl:cons ':header (header msg))
    (cl:cons ':angleX (angleX msg))
    (cl:cons ':angleY (angleY msg))
    (cl:cons ':gyroX (gyroX msg))
    (cl:cons ':gyroY (gyroY msg))
    (cl:cons ':accelX (accelX msg))
    (cl:cons ':accelY (accelY msg))
    (cl:cons ':accelZ (accelZ msg))
))
