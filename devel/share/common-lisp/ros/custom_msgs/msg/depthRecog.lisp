; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude depthRecog.msg.html

(cl:defclass <depthRecog> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (vector
    :reader vector
    :initarg :vector
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass depthRecog (<depthRecog>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <depthRecog>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'depthRecog)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<depthRecog> is deprecated: use custom_msgs-msg:depthRecog instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <depthRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:header-val is deprecated.  Use custom_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <depthRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:point-val is deprecated.  Use custom_msgs-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'vector-val :lambda-list '(m))
(cl:defmethod vector-val ((m <depthRecog>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:vector-val is deprecated.  Use custom_msgs-msg:vector instead.")
  (vector m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <depthRecog>) ostream)
  "Serializes a message object of type '<depthRecog>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vector) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <depthRecog>) istream)
  "Deserializes a message object of type '<depthRecog>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vector) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<depthRecog>)))
  "Returns string type for a message object of type '<depthRecog>"
  "custom_msgs/depthRecog")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'depthRecog)))
  "Returns string type for a message object of type 'depthRecog"
  "custom_msgs/depthRecog")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<depthRecog>)))
  "Returns md5sum for a message object of type '<depthRecog>"
  "b4d4e89e36c63a48672ef02dabdec610")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'depthRecog)))
  "Returns md5sum for a message object of type 'depthRecog"
  "b4d4e89e36c63a48672ef02dabdec610")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<depthRecog>)))
  "Returns full string definition for message of type '<depthRecog>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point point~%geometry_msgs/Point vector~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'depthRecog)))
  "Returns full string definition for message of type 'depthRecog"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point point~%geometry_msgs/Point vector~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <depthRecog>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vector))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <depthRecog>))
  "Converts a ROS message object to a list"
  (cl:list 'depthRecog
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
    (cl:cons ':vector (vector msg))
))
