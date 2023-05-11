;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::ImgRecog)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'ImgRecog (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::IMGRECOG")
  (make-package "CUSTOM_MSGS::IMGRECOG"))

(in-package "ROS")
;;//! \htmlinclude ImgRecog.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msgs::ImgRecog
  :super ros::object
  :slots (_header _detected_cnt _detected_num _point ))

(defmethod custom_msgs::ImgRecog
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected_cnt __detected_cnt) 0)
    ((:detected_num __detected_num) (make-array 0 :initial-element 0 :element-type :char))
    ((:point __point) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected_cnt (round __detected_cnt))
   (setq _detected_num __detected_num)
   (setq _point __point)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detected_cnt
   (&optional __detected_cnt)
   (if __detected_cnt (setq _detected_cnt __detected_cnt)) _detected_cnt)
  (:detected_num
   (&optional __detected_num)
   (if __detected_num (setq _detected_num __detected_num)) _detected_num)
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
    ;; uint8 _detected_cnt
    1
    ;; uint8[] _detected_num
    (* 1    (length _detected_num)) 4
    ;; geometry_msgs/Point[] _point
    (apply #'+ (send-all _point :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _detected_cnt
       (write-byte _detected_cnt s)
     ;; uint8[] _detected_num
     (write-long (length _detected_num) s)
     (princ _detected_num s)
     ;; geometry_msgs/Point[] _point
     (write-long (length _point) s)
     (dolist (elem _point)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _detected_cnt
     (setq _detected_cnt (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _detected_num
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _detected_num (make-array n :element-type :char))
     (replace _detected_num buf :start2 ptr-) (incf ptr- n))
   ;; geometry_msgs/Point[] _point
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _point)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get custom_msgs::ImgRecog :md5sum-) "3cc7bc92017f3829a0d9445ce06b3944")
(setf (get custom_msgs::ImgRecog :datatype-) "custom_msgs/ImgRecog")
(setf (get custom_msgs::ImgRecog :definition-)
      "std_msgs/Header header

uint8 detected_cnt
uint8[] detected_num


geometry_msgs/Point[] point

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



(provide :custom_msgs/ImgRecog "3cc7bc92017f3829a0d9445ce06b3944")


