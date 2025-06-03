
(cl:in-package :asdf)

(defsystem "motion_capture_ik-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :motion_capture_ik-msg
)
  :components ((:file "_package")
    (:file "changeArmCtrlMode" :depends-on ("_package_changeArmCtrlMode"))
    (:file "_package_changeArmCtrlMode" :depends-on ("_package"))
    (:file "changeArmCtrlModeKuavo" :depends-on ("_package_changeArmCtrlModeKuavo"))
    (:file "_package_changeArmCtrlModeKuavo" :depends-on ("_package"))
    (:file "fkSrv" :depends-on ("_package_fkSrv"))
    (:file "_package_fkSrv" :depends-on ("_package"))
    (:file "twoArmHandPoseCmdSrv" :depends-on ("_package_twoArmHandPoseCmdSrv"))
    (:file "_package_twoArmHandPoseCmdSrv" :depends-on ("_package"))
  ))