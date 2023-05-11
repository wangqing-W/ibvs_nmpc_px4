; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude ParkingRecog.msg.html

(cl:defclass <ParkingRecog> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass ParkingRecog (<ParkingRecog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParkingRecog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParkingRecog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<ParkingRecog> is deprecated: use custom_msgs-msg:ParkingRecog instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ParkingRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <ParkingRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:point-val is deprecated.  Use custom_msgs-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParkingRecog>) ostream)
  "Serializes a message object of type '<ParkingRecog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParkingRecog>) istream)
  "Deserializes a message object of type '<ParkingRecog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParkingRecog>)))
  "Returns string type for a message object of type '<ParkingRecog>"
  "custom_msgs/ParkingRecog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParkingRecog)))
  "Returns string type for a message object of type 'ParkingRecog"
  "custom_msgs/ParkingRecog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParkingRecog>)))
  "Returns md5sum for a message object of type '<ParkingRecog>"
  "c63aecb41bfdfd6b7e1fac37c7cbe7bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParkingRecog)))
  "Returns md5sum for a message object of type 'ParkingRecog"
  "c63aecb41bfdfd6b7e1fac37c7cbe7bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParkingRecog>)))
  "Returns full string definition for message of type '<ParkingRecog>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParkingRecog)))
  "Returns full string definition for message of type 'ParkingRecog"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParkingRecog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParkingRecog>))
  "Converts a ROS message object to a list"
  (cl:list 'ParkingRecog
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
))
