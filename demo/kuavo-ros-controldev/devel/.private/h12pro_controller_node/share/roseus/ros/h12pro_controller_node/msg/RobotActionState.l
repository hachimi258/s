;; Auto-generated. Do not edit!


(when (boundp 'h12pro_controller_node::RobotActionState)
  (if (not (find-package "H12PRO_CONTROLLER_NODE"))
    (make-package "H12PRO_CONTROLLER_NODE"))
  (shadow 'RobotActionState (find-package "H12PRO_CONTROLLER_NODE")))
(unless (find-package "H12PRO_CONTROLLER_NODE::ROBOTACTIONSTATE")
  (make-package "H12PRO_CONTROLLER_NODE::ROBOTACTIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotActionState.msg.html


(defclass h12pro_controller_node::RobotActionState
  :super ros::object
  :slots (_state ))

(defmethod h12pro_controller_node::RobotActionState
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get h12pro_controller_node::RobotActionState :md5sum-) "800f34bc468def1d86e2d42bea5648c0")
(setf (get h12pro_controller_node::RobotActionState :datatype-) "h12pro_controller_node/RobotActionState")
(setf (get h12pro_controller_node::RobotActionState :definition-)
      "uint8 state  
# 0:失败 ()
# 1:执行中
# 2:成功
")



(provide :h12pro_controller_node/RobotActionState "800f34bc468def1d86e2d42bea5648c0")


