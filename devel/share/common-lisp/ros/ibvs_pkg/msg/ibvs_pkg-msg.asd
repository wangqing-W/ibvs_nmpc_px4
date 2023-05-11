
(cl:in-package :asdf)

(defsystem "ibvs_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Marker" :depends-on ("_package_Marker"))
    (:file "_package_Marker" :depends-on ("_package"))
    (:file "point_xyz" :depends-on ("_package_point_xyz"))
    (:file "_package_point_xyz" :depends-on ("_package"))
    (:file "xyzyawVel" :depends-on ("_package_xyzyawVel"))
    (:file "_package_xyzyawVel" :depends-on ("_package"))
  ))