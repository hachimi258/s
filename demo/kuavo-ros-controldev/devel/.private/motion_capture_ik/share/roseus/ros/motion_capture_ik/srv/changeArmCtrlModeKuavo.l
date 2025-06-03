;; Auto-generated. Do not edit!


(when (boundp 'motion_capture_ik::changeArmCtrlModeKuavo)
  (if (not (find-package "MOTION_CAPTURE_IK"))
    (make-package "MOTION_CAPTURE_IK"))
  (shadow 'changeArmCtrlModeKuavo (find-package "MOTION_CAPTURE_IK")))
(unless (find-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVO")
  (make-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVO"))
(unless (find-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVOREQUEST")
  (make-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVOREQUEST"))
(unless (find-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVORESPONSE")
  (make-package "MOTION_CAPTURE_IK::CHANGEARMCTRLMODEKUAVORESPONSE"))

(in-package "ROS")





(defclass motion_capture_ik::changeArmCtrlModeKuavoRequest
  :super ros::object
  :slots (_control_mode ))

(defmethod motion_capture_ik::changeArmCtrlModeKuavoRequest
  (:init
   (&key
    ((:control_mode __control_mode) nil)
    )
   (send-super :init)
   (setq _control_mode __control_mode)
   self)
  (:control_mode
   (&optional (__control_mode :null))
   (if (not (eq __control_mode :null)) (setq _control_mode __control_mode)) _control_mode)
  (:serialization-length
   ()
   (+
    ;; bool _control_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _control_mode
       (if _control_mode (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _control_mode
     (setq _control_mode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass motion_capture_ik::changeArmCtrlModeKuavoResponse
  :super ros::object
  :slots (_result ))

(defmethod motion_capture_ik::changeArmCtrlModeKuavoResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass motion_capture_ik::changeArmCtrlModeKuavo
  :super ros::object
  :slots ())

(setf (get motion_capture_ik::changeArmCtrlModeKuavo :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get motion_capture_ik::changeArmCtrlModeKuavo :datatype-) "motion_capture_ik/changeArmCtrlModeKuavo")
(setf (get motion_capture_ik::changeArmCtrlModeKuavo :request) motion_capture_ik::changeArmCtrlModeKuavoRequest)
(setf (get motion_capture_ik::changeArmCtrlModeKuavo :response) motion_capture_ik::changeArmCtrlModeKuavoResponse)

(defmethod motion_capture_ik::changeArmCtrlModeKuavoRequest
  (:response () (instance motion_capture_ik::changeArmCtrlModeKuavoResponse :init)))

(setf (get motion_capture_ik::changeArmCtrlModeKuavoRequest :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get motion_capture_ik::changeArmCtrlModeKuavoRequest :datatype-) "motion_capture_ik/changeArmCtrlModeKuavoRequest")
(setf (get motion_capture_ik::changeArmCtrlModeKuavoRequest :definition-)
      "bool control_mode
---
bool result

")

(setf (get motion_capture_ik::changeArmCtrlModeKuavoResponse :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get motion_capture_ik::changeArmCtrlModeKuavoResponse :datatype-) "motion_capture_ik/changeArmCtrlModeKuavoResponse")
(setf (get motion_capture_ik::changeArmCtrlModeKuavoResponse :definition-)
      "bool control_mode
---
bool result

")



(provide :motion_capture_ik/changeArmCtrlModeKuavo "f89438f9d6f48f748eabe64775a22261")


