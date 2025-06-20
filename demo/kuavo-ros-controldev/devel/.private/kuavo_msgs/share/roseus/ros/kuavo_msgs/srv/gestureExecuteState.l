;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::gestureExecuteState)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'gestureExecuteState (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTESTATE")
  (make-package "KUAVO_MSGS::GESTUREEXECUTESTATE"))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTESTATEREQUEST")
  (make-package "KUAVO_MSGS::GESTUREEXECUTESTATEREQUEST"))
(unless (find-package "KUAVO_MSGS::GESTUREEXECUTESTATERESPONSE")
  (make-package "KUAVO_MSGS::GESTUREEXECUTESTATERESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::gestureExecuteStateRequest
  :super ros::object
  :slots ())

(defmethod kuavo_msgs::gestureExecuteStateRequest
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

(defclass kuavo_msgs::gestureExecuteStateResponse
  :super ros::object
  :slots (_is_executing _message ))

(defmethod kuavo_msgs::gestureExecuteStateResponse
  (:init
   (&key
    ((:is_executing __is_executing) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _is_executing __is_executing)
   (setq _message (string __message))
   self)
  (:is_executing
   (&optional (__is_executing :null))
   (if (not (eq __is_executing :null)) (setq _is_executing __is_executing)) _is_executing)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _is_executing
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_executing
       (if _is_executing (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_executing
     (setq _is_executing (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_msgs::gestureExecuteState
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::gestureExecuteState :md5sum-) "340cb7a19883494e0e422f2964927053")
(setf (get kuavo_msgs::gestureExecuteState :datatype-) "kuavo_msgs/gestureExecuteState")
(setf (get kuavo_msgs::gestureExecuteState :request) kuavo_msgs::gestureExecuteStateRequest)
(setf (get kuavo_msgs::gestureExecuteState :response) kuavo_msgs::gestureExecuteStateResponse)

(defmethod kuavo_msgs::gestureExecuteStateRequest
  (:response () (instance kuavo_msgs::gestureExecuteStateResponse :init)))

(setf (get kuavo_msgs::gestureExecuteStateRequest :md5sum-) "340cb7a19883494e0e422f2964927053")
(setf (get kuavo_msgs::gestureExecuteStateRequest :datatype-) "kuavo_msgs/gestureExecuteStateRequest")
(setf (get kuavo_msgs::gestureExecuteStateRequest :definition-)
      "# This service checks the status of a task.
# It is used to determine whether a task is currently executing and provides an additional message.

# Request:
# (No request fields for this service)

# Response:
# bool is_executing  # Indicates whether the task is currently executing.
# string message     # A message providing additional information (e.g., status details or error messages).

---
bool is_executing
string message
")

(setf (get kuavo_msgs::gestureExecuteStateResponse :md5sum-) "340cb7a19883494e0e422f2964927053")
(setf (get kuavo_msgs::gestureExecuteStateResponse :datatype-) "kuavo_msgs/gestureExecuteStateResponse")
(setf (get kuavo_msgs::gestureExecuteStateResponse :definition-)
      "# This service checks the status of a task.
# It is used to determine whether a task is currently executing and provides an additional message.

# Request:
# (No request fields for this service)

# Response:
# bool is_executing  # Indicates whether the task is currently executing.
# string message     # A message providing additional information (e.g., status details or error messages).

---
bool is_executing
string message
")



(provide :kuavo_msgs/gestureExecuteState "340cb7a19883494e0e422f2964927053")


