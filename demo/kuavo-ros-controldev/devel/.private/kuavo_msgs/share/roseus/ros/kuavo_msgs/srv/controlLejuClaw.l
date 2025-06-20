;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::controlLejuClaw)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'controlLejuClaw (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::CONTROLLEJUCLAW")
  (make-package "KUAVO_MSGS::CONTROLLEJUCLAW"))
(unless (find-package "KUAVO_MSGS::CONTROLLEJUCLAWREQUEST")
  (make-package "KUAVO_MSGS::CONTROLLEJUCLAWREQUEST"))
(unless (find-package "KUAVO_MSGS::CONTROLLEJUCLAWRESPONSE")
  (make-package "KUAVO_MSGS::CONTROLLEJUCLAWRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::controlLejuClawRequest
  :super ros::object
  :slots (_data ))

(defmethod kuavo_msgs::controlLejuClawRequest
  (:init
   (&key
    ((:data __data) (instance kuavo_msgs::endEffectorData :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; kuavo_msgs/endEffectorData _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuavo_msgs/endEffectorData _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuavo_msgs/endEffectorData _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass kuavo_msgs::controlLejuClawResponse
  :super ros::object
  :slots (_success _message ))

(defmethod kuavo_msgs::controlLejuClawResponse
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

(defclass kuavo_msgs::controlLejuClaw
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::controlLejuClaw :md5sum-) "674277f611b34c602b5afcc4b45849d1")
(setf (get kuavo_msgs::controlLejuClaw :datatype-) "kuavo_msgs/controlLejuClaw")
(setf (get kuavo_msgs::controlLejuClaw :request) kuavo_msgs::controlLejuClawRequest)
(setf (get kuavo_msgs::controlLejuClaw :response) kuavo_msgs::controlLejuClawResponse)

(defmethod kuavo_msgs::controlLejuClawRequest
  (:response () (instance kuavo_msgs::controlLejuClawResponse :init)))

(setf (get kuavo_msgs::controlLejuClawRequest :md5sum-) "674277f611b34c602b5afcc4b45849d1")
(setf (get kuavo_msgs::controlLejuClawRequest :datatype-) "kuavo_msgs/controlLejuClawRequest")
(setf (get kuavo_msgs::controlLejuClawRequest :definition-)
      "
# kuavo_msgs/endEffectorData:
# string[] name  
# float64[] position
# float64[] velocity  
# float64[] effort
# 
# ** For the Service Notes **
# 
# name     : 'left_claw' , 'right_claw'
# position : 0 ~ 100, the percentage of the claw's opening angle
#            0: closed, 100: open   
# velocity : 0 ~ 100, if size is 0, will use default `50.0`.
# effort   : torque/current, better 1A ~ 2A, if size is 0, will use default `1.0`.
# 
# ** Example **
# Request:
# data:
#   - name: ['left_claw', 'right_claw']
#     position: [20.0, 20.0]
#     velocity: [50.0, 50.0]
#     effort: [1.0, 1.0]
#
# Response:
# success: True/False, call service success or not.
# message: 'success'
kuavo_msgs/endEffectorData data

================================================================================
MSG: kuavo_msgs/endEffectorData
string[] name  
float64[] position
float64[] velocity  
float64[] effort
---
bool success
string message

")

(setf (get kuavo_msgs::controlLejuClawResponse :md5sum-) "674277f611b34c602b5afcc4b45849d1")
(setf (get kuavo_msgs::controlLejuClawResponse :datatype-) "kuavo_msgs/controlLejuClawResponse")
(setf (get kuavo_msgs::controlLejuClawResponse :definition-)
      "
# kuavo_msgs/endEffectorData:
# string[] name  
# float64[] position
# float64[] velocity  
# float64[] effort
# 
# ** For the Service Notes **
# 
# name     : 'left_claw' , 'right_claw'
# position : 0 ~ 100, the percentage of the claw's opening angle
#            0: closed, 100: open   
# velocity : 0 ~ 100, if size is 0, will use default `50.0`.
# effort   : torque/current, better 1A ~ 2A, if size is 0, will use default `1.0`.
# 
# ** Example **
# Request:
# data:
#   - name: ['left_claw', 'right_claw']
#     position: [20.0, 20.0]
#     velocity: [50.0, 50.0]
#     effort: [1.0, 1.0]
#
# Response:
# success: True/False, call service success or not.
# message: 'success'
kuavo_msgs/endEffectorData data

================================================================================
MSG: kuavo_msgs/endEffectorData
string[] name  
float64[] position
float64[] velocity  
float64[] effort
---
bool success
string message

")



(provide :kuavo_msgs/controlLejuClaw "674277f611b34c602b5afcc4b45849d1")


