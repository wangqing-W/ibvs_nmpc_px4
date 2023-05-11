; Auto-generated. Do not edit!


(cl:in-package ibvs_pkg-msg)


;//! \htmlinclude point_xyz.msg.html

(cl:defclass <point_xyz> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (p1_x
    :reader p1_x
    :initarg :p1_x
    :type cl:float
    :initform 0.0)
   (p1_y
    :reader p1_y
    :initarg :p1_y
    :type cl:float
    :initform 0.0)
   (p1_z
    :reader p1_z
    :initarg :p1_z
    :type cl:float
    :initform 0.0)
   (p2_x
    :reader p2_x
    :initarg :p2_x
    :type cl:float
    :initform 0.0)
   (p2_y
    :reader p2_y
    :initarg :p2_y
    :type cl:float
    :initform 0.0)
   (p2_z
    :reader p2_z
    :initarg :p2_z
    :type cl:float
    :initform 0.0)
   (p3_x
    :reader p3_x
    :initarg :p3_x
    :type cl:float
    :initform 0.0)
   (p3_y
    :reader p3_y
    :initarg :p3_y
    :type cl:float
    :initform 0.0)
   (p3_z
    :reader p3_z
    :initarg :p3_z
    :type cl:float
    :initform 0.0)
   (p4_x
    :reader p4_x
    :initarg :p4_x
    :type cl:float
    :initform 0.0)
   (p4_y
    :reader p4_y
    :initarg :p4_y
    :type cl:float
    :initform 0.0)
   (p4_z
    :reader p4_z
    :initarg :p4_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass point_xyz (<point_xyz>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <point_xyz>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'point_xyz)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ibvs_pkg-msg:<point_xyz> is deprecated: use ibvs_pkg-msg:point_xyz instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:header-val is deprecated.  Use ibvs_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'p1_x-val :lambda-list '(m))
(cl:defmethod p1_x-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p1_x-val is deprecated.  Use ibvs_pkg-msg:p1_x instead.")
  (p1_x m))

(cl:ensure-generic-function 'p1_y-val :lambda-list '(m))
(cl:defmethod p1_y-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p1_y-val is deprecated.  Use ibvs_pkg-msg:p1_y instead.")
  (p1_y m))

(cl:ensure-generic-function 'p1_z-val :lambda-list '(m))
(cl:defmethod p1_z-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p1_z-val is deprecated.  Use ibvs_pkg-msg:p1_z instead.")
  (p1_z m))

(cl:ensure-generic-function 'p2_x-val :lambda-list '(m))
(cl:defmethod p2_x-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p2_x-val is deprecated.  Use ibvs_pkg-msg:p2_x instead.")
  (p2_x m))

(cl:ensure-generic-function 'p2_y-val :lambda-list '(m))
(cl:defmethod p2_y-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p2_y-val is deprecated.  Use ibvs_pkg-msg:p2_y instead.")
  (p2_y m))

(cl:ensure-generic-function 'p2_z-val :lambda-list '(m))
(cl:defmethod p2_z-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p2_z-val is deprecated.  Use ibvs_pkg-msg:p2_z instead.")
  (p2_z m))

(cl:ensure-generic-function 'p3_x-val :lambda-list '(m))
(cl:defmethod p3_x-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p3_x-val is deprecated.  Use ibvs_pkg-msg:p3_x instead.")
  (p3_x m))

(cl:ensure-generic-function 'p3_y-val :lambda-list '(m))
(cl:defmethod p3_y-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p3_y-val is deprecated.  Use ibvs_pkg-msg:p3_y instead.")
  (p3_y m))

(cl:ensure-generic-function 'p3_z-val :lambda-list '(m))
(cl:defmethod p3_z-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p3_z-val is deprecated.  Use ibvs_pkg-msg:p3_z instead.")
  (p3_z m))

(cl:ensure-generic-function 'p4_x-val :lambda-list '(m))
(cl:defmethod p4_x-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p4_x-val is deprecated.  Use ibvs_pkg-msg:p4_x instead.")
  (p4_x m))

(cl:ensure-generic-function 'p4_y-val :lambda-list '(m))
(cl:defmethod p4_y-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p4_y-val is deprecated.  Use ibvs_pkg-msg:p4_y instead.")
  (p4_y m))

(cl:ensure-generic-function 'p4_z-val :lambda-list '(m))
(cl:defmethod p4_z-val ((m <point_xyz>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ibvs_pkg-msg:p4_z-val is deprecated.  Use ibvs_pkg-msg:p4_z instead.")
  (p4_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <point_xyz>) ostream)
  "Serializes a message object of type '<point_xyz>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p1_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p1_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p1_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p2_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p2_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p2_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p3_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p3_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p3_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p4_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p4_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p4_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <point_xyz>) istream)
  "Deserializes a message object of type '<point_xyz>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p1_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p1_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p1_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p2_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p2_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p2_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p3_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p3_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p3_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p4_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p4_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p4_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<point_xyz>)))
  "Returns string type for a message object of type '<point_xyz>"
  "ibvs_pkg/point_xyz")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'point_xyz)))
  "Returns string type for a message object of type 'point_xyz"
  "ibvs_pkg/point_xyz")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<point_xyz>)))
  "Returns md5sum for a message object of type '<point_xyz>"
  "3fec7e78a909523b59777fdf937fe045")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'point_xyz)))
  "Returns md5sum for a message object of type 'point_xyz"
  "3fec7e78a909523b59777fdf937fe045")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<point_xyz>)))
  "Returns full string definition for message of type '<point_xyz>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 p1_x~%float32 p1_y~%float32 p1_z~%float32 p2_x~%float32 p2_y~%float32 p2_z~%float32 p3_x~%float32 p3_y~%float32 p3_z~%float32 p4_x~%float32 p4_y~%float32 p4_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'point_xyz)))
  "Returns full string definition for message of type 'point_xyz"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 p1_x~%float32 p1_y~%float32 p1_z~%float32 p2_x~%float32 p2_y~%float32 p2_z~%float32 p3_x~%float32 p3_y~%float32 p3_z~%float32 p4_x~%float32 p4_y~%float32 p4_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <point_xyz>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <point_xyz>))
  "Converts a ROS message object to a list"
  (cl:list 'point_xyz
    (cl:cons ':header (header msg))
    (cl:cons ':p1_x (p1_x msg))
    (cl:cons ':p1_y (p1_y msg))
    (cl:cons ':p1_z (p1_z msg))
    (cl:cons ':p2_x (p2_x msg))
    (cl:cons ':p2_y (p2_y msg))
    (cl:cons ':p2_z (p2_z msg))
    (cl:cons ':p3_x (p3_x msg))
    (cl:cons ':p3_y (p3_y msg))
    (cl:cons ':p3_z (p3_z msg))
    (cl:cons ':p4_x (p4_x msg))
    (cl:cons ':p4_y (p4_y msg))
    (cl:cons ':p4_z (p4_z msg))
))
