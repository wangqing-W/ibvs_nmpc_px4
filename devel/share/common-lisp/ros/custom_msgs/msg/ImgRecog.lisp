; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude ImgRecog.msg.html

(cl:defclass <ImgRecog> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detected_cnt
    :reader detected_cnt
    :initarg :detected_cnt
    :type cl:fixnum
    :initform 0)
   (detected_num
    :reader detected_num
    :initarg :detected_num
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (point
    :reader point
    :initarg :point
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass ImgRecog (<ImgRecog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImgRecog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImgRecog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<ImgRecog> is deprecated: use custom_msgs-msg:ImgRecog instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ImgRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detected_cnt-val :lambda-list '(m))
(cl:defmethod detected_cnt-val ((m <ImgRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:detected_cnt-val is deprecated.  Use custom_msgs-msg:detected_cnt instead.")
  (detected_cnt m))

(cl:ensure-generic-function 'detected_num-val :lambda-list '(m))
(cl:defmethod detected_num-val ((m <ImgRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:detected_num-val is deprecated.  Use custom_msgs-msg:detected_num instead.")
  (detected_num m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <ImgRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:point-val is deprecated.  Use custom_msgs-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImgRecog>) ostream)
  "Serializes a message object of type '<ImgRecog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detected_cnt)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detected_num))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'detected_num))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'point))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImgRecog>) istream)
  "Deserializes a message object of type '<ImgRecog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detected_cnt)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detected_num) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detected_num)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImgRecog>)))
  "Returns string type for a message object of type '<ImgRecog>"
  "custom_msgs/ImgRecog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImgRecog)))
  "Returns string type for a message object of type 'ImgRecog"
  "custom_msgs/ImgRecog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImgRecog>)))
  "Returns md5sum for a message object of type '<ImgRecog>"
  "3cc7bc92017f3829a0d9445ce06b3944")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImgRecog)))
  "Returns md5sum for a message object of type 'ImgRecog"
  "3cc7bc92017f3829a0d9445ce06b3944")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImgRecog>)))
  "Returns full string definition for message of type '<ImgRecog>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 detected_cnt~%uint8[] detected_num~%~%~%geometry_msgs/Point[] point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImgRecog)))
  "Returns full string definition for message of type 'ImgRecog"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 detected_cnt~%uint8[] detected_num~%~%~%geometry_msgs/Point[] point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImgRecog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detected_num) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImgRecog>))
  "Converts a ROS message object to a list"
  (cl:list 'ImgRecog
    (cl:cons ':header (header msg))
    (cl:cons ':detected_cnt (detected_cnt msg))
    (cl:cons ':detected_num (detected_num msg))
    (cl:cons ':point (point msg))
))
