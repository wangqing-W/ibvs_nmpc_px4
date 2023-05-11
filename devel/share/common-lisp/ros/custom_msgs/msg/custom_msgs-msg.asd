
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CircleRecog" :depends-on ("_package_CircleRecog"))
    (:file "_package_CircleRecog" :depends-on ("_package"))
    (:file "ControlCommand" :depends-on ("_package_ControlCommand"))
    (:file "_package_ControlCommand" :depends-on ("_package"))
    (:file "FlatTarget" :depends-on ("_package_FlatTarget"))
    (:file "_package_FlatTarget" :depends-on ("_package"))
    (:file "ImgRecog" :depends-on ("_package_ImgRecog"))
    (:file "_package_ImgRecog" :depends-on ("_package"))
    (:file "ParkingRecog" :depends-on ("_package_ParkingRecog"))
    (:file "_package_ParkingRecog" :depends-on ("_package"))
    (:file "PurposeWp" :depends-on ("_package_PurposeWp"))
    (:file "_package_PurposeWp" :depends-on ("_package"))
    (:file "depthRecog" :depends-on ("_package_depthRecog"))
    (:file "_package_depthRecog" :depends-on ("_package"))
  ))