;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::ParkingRecog)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'ParkingRecog (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::PARKINGRECOG")
  (make-package "CUSTOM_MSGS::PARKINGRECOG"))

(in-package "ROS")
;;//! \htmlinclude ParkingRecog.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msgs::ParkingRecog
  :super ros::object
  :slots (_header _point ))

(defmethod custom_msgs::ParkingRecog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get custom_msgs::ParkingRecog :md5sum-) "c63aecb41bfdfd6b7e1fac37c7cbe7bf")
(setf (get custom_msgs::ParkingRecog :datatype-) "custom_msgs/ParkingRecog")
(setf (get custom_msgs::ParkingRecog :definition-)
      "std_msgs/Header header

geometry_msgs/Point point

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :custom_msgs/ParkingRecog "c63aecb41bfdfd6b7e1fac37c7cbe7bf")


