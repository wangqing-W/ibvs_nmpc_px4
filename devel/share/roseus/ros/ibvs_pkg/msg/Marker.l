;; Auto-generated. Do not edit!


(when (boundp 'ibvs_pkg::Marker)
  (if (not (find-package "IBVS_PKG"))
    (make-package "IBVS_PKG"))
  (shadow 'Marker (find-package "IBVS_PKG")))
(unless (find-package "IBVS_PKG::MARKER")
  (make-package "IBVS_PKG::MARKER"))

(in-package "ROS")
;;//! \htmlinclude Marker.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ibvs_pkg::Marker
  :super ros::object
  :slots (_header _marker_desired_x _marker_current_x _marker_current_y _marker_desired_y ))

(defmethod ibvs_pkg::Marker
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:marker_desired_x __marker_desired_x) (make-array 0 :initial-element 0 :element-type :integer))
    ((:marker_current_x __marker_current_x) (make-array 0 :initial-element 0 :element-type :integer))
    ((:marker_current_y __marker_current_y) (make-array 0 :initial-element 0 :element-type :integer))
    ((:marker_desired_y __marker_desired_y) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _marker_desired_x __marker_desired_x)
   (setq _marker_current_x __marker_current_x)
   (setq _marker_current_y __marker_current_y)
   (setq _marker_desired_y __marker_desired_y)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:marker_desired_x
   (&optional __marker_desired_x)
   (if __marker_desired_x (setq _marker_desired_x __marker_desired_x)) _marker_desired_x)
  (:marker_current_x
   (&optional __marker_current_x)
   (if __marker_current_x (setq _marker_current_x __marker_current_x)) _marker_current_x)
  (:marker_current_y
   (&optional __marker_current_y)
   (if __marker_current_y (setq _marker_current_y __marker_current_y)) _marker_current_y)
  (:marker_desired_y
   (&optional __marker_desired_y)
   (if __marker_desired_y (setq _marker_desired_y __marker_desired_y)) _marker_desired_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _marker_desired_x
    (* 4    (length _marker_desired_x)) 4
    ;; int32[] _marker_current_x
    (* 4    (length _marker_current_x)) 4
    ;; int32[] _marker_current_y
    (* 4    (length _marker_current_y)) 4
    ;; int32[] _marker_desired_y
    (* 4    (length _marker_desired_y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _marker_desired_x
     (write-long (length _marker_desired_x) s)
     (dotimes (i (length _marker_desired_x))
       (write-long (elt _marker_desired_x i) s)
       )
     ;; int32[] _marker_current_x
     (write-long (length _marker_current_x) s)
     (dotimes (i (length _marker_current_x))
       (write-long (elt _marker_current_x i) s)
       )
     ;; int32[] _marker_current_y
     (write-long (length _marker_current_y) s)
     (dotimes (i (length _marker_current_y))
       (write-long (elt _marker_current_y i) s)
       )
     ;; int32[] _marker_desired_y
     (write-long (length _marker_desired_y) s)
     (dotimes (i (length _marker_desired_y))
       (write-long (elt _marker_desired_y i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _marker_desired_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_desired_x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _marker_desired_x i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _marker_current_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_current_x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _marker_current_x i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _marker_current_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_current_y (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _marker_current_y i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _marker_desired_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _marker_desired_y (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _marker_desired_y i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get ibvs_pkg::Marker :md5sum-) "ac084a991c16309650259dd1961a407b")
(setf (get ibvs_pkg::Marker :datatype-) "ibvs_pkg/Marker")
(setf (get ibvs_pkg::Marker :definition-)
      "std_msgs/Header header

int32[] marker_desired_x
int32[] marker_current_x
int32[] marker_current_y
int32[] marker_desired_y
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



(provide :ibvs_pkg/Marker "ac084a991c16309650259dd1961a407b")


