;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::PurposeWp)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'PurposeWp (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::PURPOSEWP")
  (make-package "CUSTOM_MSGS::PURPOSEWP"))

(in-package "ROS")
;;//! \htmlinclude PurposeWp.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msgs::PurposeWp
  :super ros::object
  :slots (_header _purpose _path ))

(defmethod custom_msgs::PurposeWp
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:purpose __purpose) 0)
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _purpose (round __purpose))
   (setq _path __path)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:purpose
   (&optional __purpose)
   (if __purpose (setq _purpose __purpose)) _purpose)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _purpose
    1
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _purpose
       (write-byte _purpose s)
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _purpose
     (setq _purpose (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(setf (get custom_msgs::PurposeWp :md5sum-) "75b00cd1cd8f634afebd0ef55df62682")
(setf (get custom_msgs::PurposeWp :datatype-) "custom_msgs/PurposeWp")
(setf (get custom_msgs::PurposeWp :definition-)
      "std_msgs/Header header

uint8 purpose
nav_msgs/Path path
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :custom_msgs/PurposeWp "75b00cd1cd8f634afebd0ef55df62682")


