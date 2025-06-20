;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::getCurrentGaitName)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'getCurrentGaitName (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GETCURRENTGAITNAME")
  (make-package "KUAVO_MSGS::GETCURRENTGAITNAME"))
(unless (find-package "KUAVO_MSGS::GETCURRENTGAITNAMEREQUEST")
  (make-package "KUAVO_MSGS::GETCURRENTGAITNAMEREQUEST"))
(unless (find-package "KUAVO_MSGS::GETCURRENTGAITNAMERESPONSE")
  (make-package "KUAVO_MSGS::GETCURRENTGAITNAMERESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::getCurrentGaitNameRequest
  :super ros::object
  :slots ())

(defmethod kuavo_msgs::getCurrentGaitNameRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass kuavo_msgs::getCurrentGaitNameResponse
  :super ros::object
  :slots (_success _gait_name ))

(defmethod kuavo_msgs::getCurrentGaitNameResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:gait_name __gait_name) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _gait_name (string __gait_name))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:gait_name
   (&optional __gait_name)
   (if __gait_name (setq _gait_name __gait_name)) _gait_name)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _gait_name
    4 (length _gait_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _gait_name
       (write-long (length _gait_name) s) (princ _gait_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _gait_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gait_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_msgs::getCurrentGaitName
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::getCurrentGaitName :md5sum-) "657f58245742fcfc53dd6ab5bfa3063e")
(setf (get kuavo_msgs::getCurrentGaitName :datatype-) "kuavo_msgs/getCurrentGaitName")
(setf (get kuavo_msgs::getCurrentGaitName :request) kuavo_msgs::getCurrentGaitNameRequest)
(setf (get kuavo_msgs::getCurrentGaitName :response) kuavo_msgs::getCurrentGaitNameResponse)

(defmethod kuavo_msgs::getCurrentGaitNameRequest
  (:response () (instance kuavo_msgs::getCurrentGaitNameResponse :init)))

(setf (get kuavo_msgs::getCurrentGaitNameRequest :md5sum-) "657f58245742fcfc53dd6ab5bfa3063e")
(setf (get kuavo_msgs::getCurrentGaitNameRequest :datatype-) "kuavo_msgs/getCurrentGaitNameRequest")
(setf (get kuavo_msgs::getCurrentGaitNameRequest :definition-)
      "# Request: Empty message, no fields needed

# Response:
# success: Whether the service call was successful
# gait_name: Name of the current gait being executed
---
bool success
string gait_name

")

(setf (get kuavo_msgs::getCurrentGaitNameResponse :md5sum-) "657f58245742fcfc53dd6ab5bfa3063e")
(setf (get kuavo_msgs::getCurrentGaitNameResponse :datatype-) "kuavo_msgs/getCurrentGaitNameResponse")
(setf (get kuavo_msgs::getCurrentGaitNameResponse :definition-)
      "# Request: Empty message, no fields needed

# Response:
# success: Whether the service call was successful
# gait_name: Name of the current gait being executed
---
bool success
string gait_name

")



(provide :kuavo_msgs/getCurrentGaitName "657f58245742fcfc53dd6ab5bfa3063e")


