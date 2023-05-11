; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude PurposeWp.msg.html

(cl:defclass <PurposeWp> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (purpose
    :reader purpose
    :initarg :purpose
    :type cl:fixnum
    :initform 0)
   (path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass PurposeWp (<PurposeWp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PurposeWp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PurposeWp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<PurposeWp> is deprecated: use custom_msgs-msg:PurposeWp instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PurposeWp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'purpose-val :lambda-list '(m))
(cl:defmethod purpose-val ((m <PurposeWp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:purpose-val is deprecated.  Use custom_msgs-msg:purpose instead.")
  (purpose m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <PurposeWp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:path-val is deprecated.  Use custom_msgs-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PurposeWp>) ostream)
  "Serializes a message object of type '<PurposeWp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'purpose)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PurposeWp>) istream)
  "Deserializes a message object of type '<PurposeWp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'purpose)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PurposeWp>)))
  "Returns string type for a message object of type '<PurposeWp>"
  "custom_msgs/PurposeWp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PurposeWp)))
  "Returns string type for a message object of type 'PurposeWp"
  "custom_msgs/PurposeWp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PurposeWp>)))
  "Returns md5sum for a message object of type '<PurposeWp>"
  "75b00cd1cd8f634afebd0ef55df62682")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PurposeWp)))
  "Returns md5sum for a message object of type 'PurposeWp"
  "75b00cd1cd8f634afebd0ef55df62682")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PurposeWp>)))
  "Returns full string definition for message of type '<PurposeWp>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 purpose~%nav_msgs/Path path~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PurposeWp)))
  "Returns full string definition for message of type 'PurposeWp"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 purpose~%nav_msgs/Path path~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PurposeWp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PurposeWp>))
  "Converts a ROS message object to a list"
  (cl:list 'PurposeWp
    (cl:cons ':header (header msg))
    (cl:cons ':purpose (purpose msg))
    (cl:cons ':path (path msg))
))
