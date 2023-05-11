;; Auto-generated. Do not edit!


(when (boundp 'ibvs_pkg::xyzyawVel)
  (if (not (find-package "IBVS_PKG"))
    (make-package "IBVS_PKG"))
  (shadow 'xyzyawVel (find-package "IBVS_PKG")))
(unless (find-package "IBVS_PKG::XYZYAWVEL")
  (make-package "IBVS_PKG::XYZYAWVEL"))

(in-package "ROS")
;;//! \htmlinclude xyzyawVel.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ibvs_pkg::xyzyawVel
  :super ros::object
  :slots (_header _Vx _Vy _Vz _Vyaw ))

(defmethod ibvs_pkg::xyzyawVel
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:Vx __Vx) 0.0)
    ((:Vy __Vy) 0.0)
    ((:Vz __Vz) 0.0)
    ((:Vyaw __Vyaw) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _Vx (float __Vx))
   (setq _Vy (float __Vy))
   (setq _Vz (float __Vz))
   (setq _Vyaw (float __Vyaw))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:Vx
   (&optional __Vx)
   (if __Vx (setq _Vx __Vx)) _Vx)
  (:Vy
   (&optional __Vy)
   (if __Vy (setq _Vy __Vy)) _Vy)
  (:Vz
   (&optional __Vz)
   (if __Vz (setq _Vz __Vz)) _Vz)
  (:Vyaw
   (&optional __Vyaw)
   (if __Vyaw (setq _Vyaw __Vyaw)) _Vyaw)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _Vx
    4
    ;; float32 _Vy
    4
    ;; float32 _Vz
    4
    ;; float32 _Vyaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _Vx
       (sys::poke _Vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Vy
       (sys::poke _Vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Vz
       (sys::poke _Vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Vyaw
       (sys::poke _Vyaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _Vx
     (setq _Vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Vy
     (setq _Vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Vz
     (setq _Vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Vyaw
     (setq _Vyaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ibvs_pkg::xyzyawVel :md5sum-) "cf984bc80b8e8684cced7bacc6b18f87")
(setf (get ibvs_pkg::xyzyawVel :datatype-) "ibvs_pkg/xyzyawVel")
(setf (get ibvs_pkg::xyzyawVel :definition-)
      "std_msgs/Header header

float32 Vx
float32 Vy
float32 Vz
float32 Vyaw
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



(provide :ibvs_pkg/xyzyawVel "cf984bc80b8e8684cced7bacc6b18f87")


