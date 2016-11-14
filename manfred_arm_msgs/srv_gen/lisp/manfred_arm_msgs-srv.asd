
(cl:in-package :asdf)

(defsystem "manfred_arm_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "Get_goal" :depends-on ("_package_Get_goal"))
    (:file "_package_Get_goal" :depends-on ("_package"))
    (:file "Get_FK" :depends-on ("_package_Get_FK"))
    (:file "_package_Get_FK" :depends-on ("_package"))
    (:file "Write_pmac" :depends-on ("_package_Write_pmac"))
    (:file "_package_Write_pmac" :depends-on ("_package"))
    (:file "Get_IK" :depends-on ("_package_Get_IK"))
    (:file "_package_Get_IK" :depends-on ("_package"))
    (:file "IK_de_quat" :depends-on ("_package_IK_de_quat"))
    (:file "_package_IK_de_quat" :depends-on ("_package"))
    (:file "Get_IK_ed" :depends-on ("_package_Get_IK_ed"))
    (:file "_package_Get_IK_ed" :depends-on ("_package"))
  ))