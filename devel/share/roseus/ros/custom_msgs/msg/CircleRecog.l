;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::CircleRecog)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'CircleRecog (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::CIRCLERECOG")
  (make-package "CUSTOM_MSGS::CIRCLERECOG"))

(in-package "ROS")
;;//! \htmlinclude CircleRecog.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msgs::CircleRecog
  :super ros::object
  :slots (_header _point _vector ))

(defmethod custom_msgs::CircleRecog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:vector __vector) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _point __point)
   (setq _vector __vector)
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
  (:vector
   (&rest __vector)
   (if (keywordp (car __vector))
       (send* _vector __vector)
     (progn
       (if __vector (setq _vector (car __vector)))
       _vector)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; geometry_msgs/Point _vector
    (send _vector :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; geometry_msgs/Point _vector
       (send _vector :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; geometry_msgs/Point _vector
     (send _vector :deserialize buf ptr-) (incf ptr- (send _vector :serialization-length))
   ;;
   self)
  )

(setf (get custom_msgs::CircleRecog :md5sum-) "b4d4e89e36c63a48672ef02dabdec610")
(setf (get custom_msgs::CircleRecog :datatype-) "custom_msgs/CircleRecog")
(setf (get custom_msgs::CircleRecog :definition-)
      "std_msgs/Header header

# 机体坐标系下的xyz
geometry_msgs/Point point
# 集体坐标系下识别到圆的单位法向量
geometry_msgs/Point vector

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



(provide :custom_msgs/CircleRecog "b4d4e89e36c63a48672ef02dabdec610")


