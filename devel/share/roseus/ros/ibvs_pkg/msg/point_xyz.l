;; Auto-generated. Do not edit!


(when (boundp 'ibvs_pkg::point_xyz)
  (if (not (find-package "IBVS_PKG"))
    (make-package "IBVS_PKG"))
  (shadow 'point_xyz (find-package "IBVS_PKG")))
(unless (find-package "IBVS_PKG::POINT_XYZ")
  (make-package "IBVS_PKG::POINT_XYZ"))

(in-package "ROS")
;;//! \htmlinclude point_xyz.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ibvs_pkg::point_xyz
  :super ros::object
  :slots (_header _p1_x _p1_y _p1_z _p2_x _p2_y _p2_z _p3_x _p3_y _p3_z _p4_x _p4_y _p4_z ))

(defmethod ibvs_pkg::point_xyz
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:p1_x __p1_x) 0.0)
    ((:p1_y __p1_y) 0.0)
    ((:p1_z __p1_z) 0.0)
    ((:p2_x __p2_x) 0.0)
    ((:p2_y __p2_y) 0.0)
    ((:p2_z __p2_z) 0.0)
    ((:p3_x __p3_x) 0.0)
    ((:p3_y __p3_y) 0.0)
    ((:p3_z __p3_z) 0.0)
    ((:p4_x __p4_x) 0.0)
    ((:p4_y __p4_y) 0.0)
    ((:p4_z __p4_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _p1_x (float __p1_x))
   (setq _p1_y (float __p1_y))
   (setq _p1_z (float __p1_z))
   (setq _p2_x (float __p2_x))
   (setq _p2_y (float __p2_y))
   (setq _p2_z (float __p2_z))
   (setq _p3_x (float __p3_x))
   (setq _p3_y (float __p3_y))
   (setq _p3_z (float __p3_z))
   (setq _p4_x (float __p4_x))
   (setq _p4_y (float __p4_y))
   (setq _p4_z (float __p4_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:p1_x
   (&optional __p1_x)
   (if __p1_x (setq _p1_x __p1_x)) _p1_x)
  (:p1_y
   (&optional __p1_y)
   (if __p1_y (setq _p1_y __p1_y)) _p1_y)
  (:p1_z
   (&optional __p1_z)
   (if __p1_z (setq _p1_z __p1_z)) _p1_z)
  (:p2_x
   (&optional __p2_x)
   (if __p2_x (setq _p2_x __p2_x)) _p2_x)
  (:p2_y
   (&optional __p2_y)
   (if __p2_y (setq _p2_y __p2_y)) _p2_y)
  (:p2_z
   (&optional __p2_z)
   (if __p2_z (setq _p2_z __p2_z)) _p2_z)
  (:p3_x
   (&optional __p3_x)
   (if __p3_x (setq _p3_x __p3_x)) _p3_x)
  (:p3_y
   (&optional __p3_y)
   (if __p3_y (setq _p3_y __p3_y)) _p3_y)
  (:p3_z
   (&optional __p3_z)
   (if __p3_z (setq _p3_z __p3_z)) _p3_z)
  (:p4_x
   (&optional __p4_x)
   (if __p4_x (setq _p4_x __p4_x)) _p4_x)
  (:p4_y
   (&optional __p4_y)
   (if __p4_y (setq _p4_y __p4_y)) _p4_y)
  (:p4_z
   (&optional __p4_z)
   (if __p4_z (setq _p4_z __p4_z)) _p4_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _p1_x
    4
    ;; float32 _p1_y
    4
    ;; float32 _p1_z
    4
    ;; float32 _p2_x
    4
    ;; float32 _p2_y
    4
    ;; float32 _p2_z
    4
    ;; float32 _p3_x
    4
    ;; float32 _p3_y
    4
    ;; float32 _p3_z
    4
    ;; float32 _p4_x
    4
    ;; float32 _p4_y
    4
    ;; float32 _p4_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _p1_x
       (sys::poke _p1_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p1_y
       (sys::poke _p1_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p1_z
       (sys::poke _p1_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p2_x
       (sys::poke _p2_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p2_y
       (sys::poke _p2_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p2_z
       (sys::poke _p2_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p3_x
       (sys::poke _p3_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p3_y
       (sys::poke _p3_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p3_z
       (sys::poke _p3_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p4_x
       (sys::poke _p4_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p4_y
       (sys::poke _p4_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _p4_z
       (sys::poke _p4_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _p1_x
     (setq _p1_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p1_y
     (setq _p1_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p1_z
     (setq _p1_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p2_x
     (setq _p2_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p2_y
     (setq _p2_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p2_z
     (setq _p2_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p3_x
     (setq _p3_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p3_y
     (setq _p3_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p3_z
     (setq _p3_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p4_x
     (setq _p4_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p4_y
     (setq _p4_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _p4_z
     (setq _p4_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ibvs_pkg::point_xyz :md5sum-) "3fec7e78a909523b59777fdf937fe045")
(setf (get ibvs_pkg::point_xyz :datatype-) "ibvs_pkg/point_xyz")
(setf (get ibvs_pkg::point_xyz :definition-)
      "std_msgs/Header header

float32 p1_x
float32 p1_y
float32 p1_z
float32 p2_x
float32 p2_y
float32 p2_z
float32 p3_x
float32 p3_y
float32 p3_z
float32 p4_x
float32 p4_y
float32 p4_z
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

")



(provide :ibvs_pkg/point_xyz "3fec7e78a909523b59777fdf937fe045")


