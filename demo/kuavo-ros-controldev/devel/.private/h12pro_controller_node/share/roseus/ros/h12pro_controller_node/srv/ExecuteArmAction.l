;; Auto-generated. Do not edit!


(when (boundp 'h12pro_controller_node::ExecuteArmAction)
  (if (not (find-package "H12PRO_CONTROLLER_NODE"))
    (make-package "H12PRO_CONTROLLER_NODE"))
  (shadow 'ExecuteArmAction (find-package "H12PRO_CONTROLLER_NODE")))
(unless (find-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTION")
  (make-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTION"))
(unless (find-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTIONREQUEST")
  (make-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTIONREQUEST"))
(unless (find-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTIONRESPONSE")
  (make-package "H12PRO_CONTROLLER_NODE::EXECUTEARMACTIONRESPONSE"))

(in-package "ROS")





(defclass h12pro_controller_node::ExecuteArmActionRequest
  :super ros::object
  :slots (_action_name ))

(defmethod h12pro_controller_node::ExecuteArmActionRequest
  (:init
   (&key
    ((:action_name __action_name) "")
    )
   (send-super :init)
   (setq _action_name (string __action_name))
   self)
  (:action_name
   (&optional __action_name)
   (if __action_name (setq _action_name __action_name)) _action_name)
  (:serialization-length
   ()
   (+
    ;; string _action_name
    4 (length _action_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action_name
       (write-long (length _action_name) s) (princ _action_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass h12pro_controller_node::ExecuteArmActionResponse
  :super ros::object
  :slots (_success _message ))

(defmethod h12pro_controller_node::ExecuteArmActionResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass h12pro_controller_node::ExecuteArmAction
  :super ros::object
  :slots ())

(setf (get h12pro_controller_node::ExecuteArmAction :md5sum-) "a29ecc17b1498502f1af3d6bb5a90a24")
(setf (get h12pro_controller_node::ExecuteArmAction :datatype-) "h12pro_controller_node/ExecuteArmAction")
(setf (get h12pro_controller_node::ExecuteArmAction :request) h12pro_controller_node::ExecuteArmActionRequest)
(setf (get h12pro_controller_node::ExecuteArmAction :response) h12pro_controller_node::ExecuteArmActionResponse)

(defmethod h12pro_controller_node::ExecuteArmActionRequest
  (:response () (instance h12pro_controller_node::ExecuteArmActionResponse :init)))

(setf (get h12pro_controller_node::ExecuteArmActionRequest :md5sum-) "a29ecc17b1498502f1af3d6bb5a90a24")
(setf (get h12pro_controller_node::ExecuteArmActionRequest :datatype-) "h12pro_controller_node/ExecuteArmActionRequest")
(setf (get h12pro_controller_node::ExecuteArmActionRequest :definition-)
      "string action_name  # 要执行的动作名称，例如 \"welcome\"
---
bool success         # 执行是否成功
string message       # 返回的消息
")

(setf (get h12pro_controller_node::ExecuteArmActionResponse :md5sum-) "a29ecc17b1498502f1af3d6bb5a90a24")
(setf (get h12pro_controller_node::ExecuteArmActionResponse :datatype-) "h12pro_controller_node/ExecuteArmActionResponse")
(setf (get h12pro_controller_node::ExecuteArmActionResponse :definition-)
      "string action_name  # 要执行的动作名称，例如 \"welcome\"
---
bool success         # 执行是否成功
string message       # 返回的消息
")



(provide :h12pro_controller_node/ExecuteArmAction "a29ecc17b1498502f1af3d6bb5a90a24")


