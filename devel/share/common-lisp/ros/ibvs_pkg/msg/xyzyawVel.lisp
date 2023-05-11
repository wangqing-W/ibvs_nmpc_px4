; Auto-generated. Do not edit!


(cl:in-package ibvs_pkg-msg)


;//! \htmlinclude xyzyawVel.msg.html

(cl:defclass <xyzyawVel> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Vx
    :reader Vx
    :initarg :Vx
    :type cl:float
    :initform 0.0)
   (Vy
    :reader Vy
    :initarg :Vy
    :type cl:float
    :initform 0.0)
   (Vz
    :reader Vz
    :initarg :Vz
    :type cl:float
    :initform 0.0)
   (Vyaw
    :reader Vyaw
    :initarg :Vyaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass xyzyawVel (<xyzyawVel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <xyzyawVel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'xyzyawVel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ibvs_pkg-msg:<xyzyawVel> is deprecated: use ibvs_pkg-msg:xyzyawVel instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <xyzyawVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:header-val is deprecated.  Use ibvs_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Vx-val :lambda-list '(m))
(cl:defmethod Vx-val ((m <xyzyawVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:Vx-val is deprecated.  Use ibvs_pkg-msg:Vx instead.")
  (Vx m))

(cl:ensure-generic-function 'Vy-val :lambda-list '(m))
(cl:defmethod Vy-val ((m <xyzyawVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:Vy-val is deprecated.  Use ibvs_pkg-msg:Vy instead.")
  (Vy m))

(cl:ensure-generic-function 'Vz-val :lambda-list '(m))
(cl:defmethod Vz-val ((m <xyzyawVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:Vz-val is deprecated.  Use ibvs_pkg-msg:Vz instead.")
  (Vz m))

(cl:ensure-generic-function 'Vyaw-val :lambda-list '(m))
(cl:defmethod Vyaw-val ((m <xyzyawVel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:Vyaw-val is deprecated.  Use ibvs_pkg-msg:Vyaw instead.")
  (Vyaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <xyzyawVel>) ostream)
  "Serializes a message object of type '<xyzyawVel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Vz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Vyaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <xyzyawVel>) istream)
  "Deserializes a message object of type '<xyzyawVel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vyaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<xyzyawVel>)))
  "Returns string type for a message object of type '<xyzyawVel>"
  "ibvs_pkg/xyzyawVel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'xyzyawVel)))
  "Returns string type for a message object of type 'xyzyawVel"
  "ibvs_pkg/xyzyawVel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<xyzyawVel>)))
  "Returns md5sum for a message object of type '<xyzyawVel>"
  "cf984bc80b8e8684cced7bacc6b18f87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'xyzyawVel)))
  "Returns md5sum for a message object of type 'xyzyawVel"
  "cf984bc80b8e8684cced7bacc6b18f87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<xyzyawVel>)))
  "Returns full string definition for message of type '<xyzyawVel>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 Vx~%float32 Vy~%float32 Vz~%float32 Vyaw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'xyzyawVel)))
  "Returns full string definition for message of type 'xyzyawVel"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 Vx~%float32 Vy~%float32 Vz~%float32 Vyaw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <xyzyawVel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <xyzyawVel>))
  "Converts a ROS message object to a list"
  (cl:list 'xyzyawVel
    (cl:cons ':header (header msg))
    (cl:cons ':Vx (Vx msg))
    (cl:cons ':Vy (Vy msg))
    (cl:cons ':Vz (Vz msg))
    (cl:cons ':Vyaw (Vyaw msg))
))
