;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::changeArmCtrlModeKuavo)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'changeArmCtrlModeKuavo (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVO")
  (make-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVO"))
(unless (find-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVOREQUEST")
  (make-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVOREQUEST"))
(unless (find-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVORESPONSE")
  (make-package "KUAVO_MSGS::CHANGEARMCTRLMODEKUAVORESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::changeArmCtrlModeKuavoRequest
  :super ros::object
  :slots (_control_mode ))

(defmethod kuavo_msgs::changeArmCtrlModeKuavoRequest
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

(defclass kuavo_msgs::changeArmCtrlModeKuavoResponse
  :super ros::object
  :slots (_result ))

(defmethod kuavo_msgs::changeArmCtrlModeKuavoResponse
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

(defclass kuavo_msgs::changeArmCtrlModeKuavo
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::changeArmCtrlModeKuavo :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get kuavo_msgs::changeArmCtrlModeKuavo :datatype-) "kuavo_msgs/changeArmCtrlModeKuavo")
(setf (get kuavo_msgs::changeArmCtrlModeKuavo :request) kuavo_msgs::changeArmCtrlModeKuavoRequest)
(setf (get kuavo_msgs::changeArmCtrlModeKuavo :response) kuavo_msgs::changeArmCtrlModeKuavoResponse)

(defmethod kuavo_msgs::changeArmCtrlModeKuavoRequest
  (:response () (instance kuavo_msgs::changeArmCtrlModeKuavoResponse :init)))

(setf (get kuavo_msgs::changeArmCtrlModeKuavoRequest :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get kuavo_msgs::changeArmCtrlModeKuavoRequest :datatype-) "kuavo_msgs/changeArmCtrlModeKuavoRequest")
(setf (get kuavo_msgs::changeArmCtrlModeKuavoRequest :definition-)
      "bool control_mode
---
bool result

")

(setf (get kuavo_msgs::changeArmCtrlModeKuavoResponse :md5sum-) "f89438f9d6f48f748eabe64775a22261")
(setf (get kuavo_msgs::changeArmCtrlModeKuavoResponse :datatype-) "kuavo_msgs/changeArmCtrlModeKuavoResponse")
(setf (get kuavo_msgs::changeArmCtrlModeKuavoResponse :definition-)
      "bool control_mode
---
bool result

")



(provide :kuavo_msgs/changeArmCtrlModeKuavo "f89438f9d6f48f748eabe64775a22261")


