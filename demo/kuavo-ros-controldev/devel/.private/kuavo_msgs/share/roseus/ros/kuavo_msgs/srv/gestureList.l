;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::gestureList)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'gestureList (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::GESTURELIST")
  (make-package "KUAVO_MSGS::GESTURELIST"))
(unless (find-package "KUAVO_MSGS::GESTURELISTREQUEST")
  (make-package "KUAVO_MSGS::GESTURELISTREQUEST"))
(unless (find-package "KUAVO_MSGS::GESTURELISTRESPONSE")
  (make-package "KUAVO_MSGS::GESTURELISTRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::gestureListRequest
  :super ros::object
  :slots ())

(defmethod kuavo_msgs::gestureListRequest
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

(defclass kuavo_msgs::gestureListResponse
  :super ros::object
  :slots (_success _gesture_count _message _gesture_infos ))

(defmethod kuavo_msgs::gestureListResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:gesture_count __gesture_count) 0)
    ((:message __message) "")
    ((:gesture_infos __gesture_infos) ())
    )
   (send-super :init)
   (setq _success __success)
   (setq _gesture_count (round __gesture_count))
   (setq _message (string __message))
   (setq _gesture_infos __gesture_infos)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:gesture_count
   (&optional __gesture_count)
   (if __gesture_count (setq _gesture_count __gesture_count)) _gesture_count)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:gesture_infos
   (&rest __gesture_infos)
   (if (keywordp (car __gesture_infos))
       (send* _gesture_infos __gesture_infos)
     (progn
       (if __gesture_infos (setq _gesture_infos (car __gesture_infos)))
       _gesture_infos)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _gesture_count
    4
    ;; string _message
    4 (length _message)
    ;; kuavo_msgs/gestureInfo[] _gesture_infos
    (apply #'+ (send-all _gesture_infos :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _gesture_count
       (write-long _gesture_count s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; kuavo_msgs/gestureInfo[] _gesture_infos
     (write-long (length _gesture_infos) s)
     (dolist (elem _gesture_infos)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _gesture_count
     (setq _gesture_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kuavo_msgs/gestureInfo[] _gesture_infos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gesture_infos (let (r) (dotimes (i n) (push (instance kuavo_msgs::gestureInfo :init) r)) r))
     (dolist (elem- _gesture_infos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass kuavo_msgs::gestureList
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::gestureList :md5sum-) "ee839a0568f441526fe05bf2e2f25572")
(setf (get kuavo_msgs::gestureList :datatype-) "kuavo_msgs/gestureList")
(setf (get kuavo_msgs::gestureList :request) kuavo_msgs::gestureListRequest)
(setf (get kuavo_msgs::gestureList :response) kuavo_msgs::gestureListResponse)

(defmethod kuavo_msgs::gestureListRequest
  (:response () (instance kuavo_msgs::gestureListResponse :init)))

(setf (get kuavo_msgs::gestureListRequest :md5sum-) "ee839a0568f441526fe05bf2e2f25572")
(setf (get kuavo_msgs::gestureListRequest :datatype-) "kuavo_msgs/gestureListRequest")
(setf (get kuavo_msgs::gestureListRequest :definition-)
      "# This service returns a list of available gestures.
# It is used to query the system for all gestures that can be recognized or performed.
#
# Request:
# No input parameters are required.
#
# Response:
# bool success                # Indicates whether the request was successful.
# int32 gesture_count         # The number of gestures returned in the list.
# string message              # A message indicating the result of the request.
# kuavo_msgs/gestureInfo[] gesture_infos # A list of gesture information, each containing the name, alias, and description of a gesture.

# Define the GestureInfo message
# string gesture_name        # The name of the gesture.
# string[] alias             # A list of aliases for the gesture.
# string description         # A description of the gesture.

---
bool success
int32 gesture_count
string message
kuavo_msgs/gestureInfo[] gesture_infos

================================================================================
MSG: kuavo_msgs/gestureInfo
# This message defines the information for a single gesture.
# It includes the name, a list of aliases, and a description of the gesture.

# The name of the gesture.
string gesture_name

# A list of aliases for the gesture. These can be alternative names or shortcuts.
string[] alias

# A description of the gesture, providing more detailed information about its purpose and usage.
")

(setf (get kuavo_msgs::gestureListResponse :md5sum-) "ee839a0568f441526fe05bf2e2f25572")
(setf (get kuavo_msgs::gestureListResponse :datatype-) "kuavo_msgs/gestureListResponse")
(setf (get kuavo_msgs::gestureListResponse :definition-)
      "# This service returns a list of available gestures.
# It is used to query the system for all gestures that can be recognized or performed.
#
# Request:
# No input parameters are required.
#
# Response:
# bool success                # Indicates whether the request was successful.
# int32 gesture_count         # The number of gestures returned in the list.
# string message              # A message indicating the result of the request.
# kuavo_msgs/gestureInfo[] gesture_infos # A list of gesture information, each containing the name, alias, and description of a gesture.

# Define the GestureInfo message
# string gesture_name        # The name of the gesture.
# string[] alias             # A list of aliases for the gesture.
# string description         # A description of the gesture.

---
bool success
int32 gesture_count
string message
kuavo_msgs/gestureInfo[] gesture_infos

================================================================================
MSG: kuavo_msgs/gestureInfo
# This message defines the information for a single gesture.
# It includes the name, a list of aliases, and a description of the gesture.

# The name of the gesture.
string gesture_name

# A list of aliases for the gesture. These can be alternative names or shortcuts.
string[] alias

# A description of the gesture, providing more detailed information about its purpose and usage.
")



(provide :kuavo_msgs/gestureList "ee839a0568f441526fe05bf2e2f25572")


