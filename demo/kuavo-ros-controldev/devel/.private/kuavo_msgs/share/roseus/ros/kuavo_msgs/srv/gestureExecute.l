;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::gestureExecute)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'gestureExecute (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTE")
  (make-package "KUAVO_MSGS::GESTUREEXECUTE"))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTEREQUEST")
  (make-package "KUAVO_MSGS::GESTUREEXECUTEREQUEST"))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTERESPONSE")
  (make-package "KUAVO_MSGS::GESTUREEXECUTERESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::gestureExecuteRequest
  :super ros::object
  :slots (_gestures ))

(defmethod kuavo_msgs::gestureExecuteRequest
  (:init
   (&key
    ((:gestures __gestures) ())
    )
   (send-super :init)
   (setq _gestures __gestures)
   self)
  (:gestures
   (&rest __gestures)
   (if (keywordp (car __gestures))
       (send* _gestures __gestures)
     (progn
       (if __gestures (setq _gestures (car __gestures)))
       _gestures)))
  (:serialization-length
   ()
   (+
    ;; kuavo_msgs/gestureTask[] _gestures
    (apply #'+ (send-all _gestures :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuavo_msgs/gestureTask[] _gestures
     (write-long (length _gestures) s)
     (dolist (elem _gestures)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuavo_msgs/gestureTask[] _gestures
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gestures (let (r) (dotimes (i n) (push (instance kuavo_msgs::gestureTask :init) r)) r))
     (dolist (elem- _gestures)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass kuavo_msgs::gestureExecuteResponse
  :super ros::object
  :slots (_success _message ))

(defmethod kuavo_msgs::gestureExecuteResponse
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

(defclass kuavo_msgs::gestureExecute
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::gestureExecute :md5sum-) "b599da36839d439975fbac8d4bfbeb7e")
(setf (get kuavo_msgs::gestureExecute :datatype-) "kuavo_msgs/gestureExecute")
(setf (get kuavo_msgs::gestureExecute :request) kuavo_msgs::gestureExecuteRequest)
(setf (get kuavo_msgs::gestureExecute :response) kuavo_msgs::gestureExecuteResponse)

(defmethod kuavo_msgs::gestureExecuteRequest
  (:response () (instance kuavo_msgs::gestureExecuteResponse :init)))

(setf (get kuavo_msgs::gestureExecuteRequest :md5sum-) "b599da36839d439975fbac8d4bfbeb7e")
(setf (get kuavo_msgs::gestureExecuteRequest :datatype-) "kuavo_msgs/gestureExecuteRequest")
(setf (get kuavo_msgs::gestureExecuteRequest :definition-)
      "# This service executes a specified gesture.
# It is used to trigger a gesture by providing its name and the side of the hand(s) to use.
#
# Request:
# kuavo_msgs/gestureTask[] gestures # An array of gestures to execute, each with a name and hand side
#
# Response:
# bool success         # Indicates whether the gesture execution was successful.
# string message       # A message providing additional information (e.g., error details if the gesture failed).

kuavo_msgs/gestureTask[] gestures

================================================================================
MSG: kuavo_msgs/gestureTask
# This message is used to specify a gesture to execute.
# The gesture is triggered by providing its name and the side of the hand(s) to use.

string gesture_name  # Name of the gesture to execute
---
bool success
string message
")

(setf (get kuavo_msgs::gestureExecuteResponse :md5sum-) "b599da36839d439975fbac8d4bfbeb7e")
(setf (get kuavo_msgs::gestureExecuteResponse :datatype-) "kuavo_msgs/gestureExecuteResponse")
(setf (get kuavo_msgs::gestureExecuteResponse :definition-)
      "# This service executes a specified gesture.
# It is used to trigger a gesture by providing its name and the side of the hand(s) to use.
#
# Request:
# kuavo_msgs/gestureTask[] gestures # An array of gestures to execute, each with a name and hand side
#
# Response:
# bool success         # Indicates whether the gesture execution was successful.
# string message       # A message providing additional information (e.g., error details if the gesture failed).

kuavo_msgs/gestureTask[] gestures

================================================================================
MSG: kuavo_msgs/gestureTask
# This message is used to specify a gesture to execute.
# The gesture is triggered by providing its name and the side of the hand(s) to use.

string gesture_name  # Name of the gesture to execute
---
bool success
string message
")



(provide :kuavo_msgs/gestureExecute "b599da36839d439975fbac8d4bfbeb7e")


