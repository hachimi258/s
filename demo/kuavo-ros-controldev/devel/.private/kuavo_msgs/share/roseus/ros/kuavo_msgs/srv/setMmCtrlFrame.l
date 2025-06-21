;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::setMmCtrlFrame)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'setMmCtrlFrame (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SETMMCTRLFRAME")
  (make-package "KUAVO_MSGS::SETMMCTRLFRAME"))
(unless (find-package "KUAVO_MSGS::SETMMCTRLFRAMEREQUEST")
  (make-package "KUAVO_MSGS::SETMMCTRLFRAMEREQUEST"))
(unless (find-package "KUAVO_MSGS::SETMMCTRLFRAMERESPONSE")
  (make-package "KUAVO_MSGS::SETMMCTRLFRAMERESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::setMmCtrlFrameRequest
  :super ros::object
  :slots (_frame ))

(defmethod kuavo_msgs::setMmCtrlFrameRequest
  (:init
   (&key
    ((:frame __frame) 0)
    )
   (send-super :init)
   (setq _frame (round __frame))
   self)
  (:frame
   (&optional __frame)
   (if __frame (setq _frame __frame)) _frame)
  (:serialization-length
   ()
   (+
    ;; int32 _frame
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _frame
       (write-long _frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _frame
     (setq _frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass kuavo_msgs::setMmCtrlFrameResponse
  :super ros::object
  :slots (_result _currentFrame _message ))

(defmethod kuavo_msgs::setMmCtrlFrameResponse
  (:init
   (&key
    ((:result __result) nil)
    ((:currentFrame __currentFrame) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _result __result)
   (setq _currentFrame (round __currentFrame))
   (setq _message (string __message))
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:currentFrame
   (&optional __currentFrame)
   (if __currentFrame (setq _currentFrame __currentFrame)) _currentFrame)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ;; int32 _currentFrame
    4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;; int32 _currentFrame
       (write-long _currentFrame s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _currentFrame
     (setq _currentFrame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_msgs::setMmCtrlFrame
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::setMmCtrlFrame :md5sum-) "696a444d2580aa682923215a4a34937c")
(setf (get kuavo_msgs::setMmCtrlFrame :datatype-) "kuavo_msgs/setMmCtrlFrame")
(setf (get kuavo_msgs::setMmCtrlFrame :request) kuavo_msgs::setMmCtrlFrameRequest)
(setf (get kuavo_msgs::setMmCtrlFrame :response) kuavo_msgs::setMmCtrlFrameResponse)

(defmethod kuavo_msgs::setMmCtrlFrameRequest
  (:response () (instance kuavo_msgs::setMmCtrlFrameResponse :init)))

(setf (get kuavo_msgs::setMmCtrlFrameRequest :md5sum-) "696a444d2580aa682923215a4a34937c")
(setf (get kuavo_msgs::setMmCtrlFrameRequest :datatype-) "kuavo_msgs/setMmCtrlFrameRequest")
(setf (get kuavo_msgs::setMmCtrlFrameRequest :definition-)
      "int32 frame
---
bool result
int32 currentFrame 
string message
")

(setf (get kuavo_msgs::setMmCtrlFrameResponse :md5sum-) "696a444d2580aa682923215a4a34937c")
(setf (get kuavo_msgs::setMmCtrlFrameResponse :datatype-) "kuavo_msgs/setMmCtrlFrameResponse")
(setf (get kuavo_msgs::setMmCtrlFrameResponse :definition-)
      "int32 frame
---
bool result
int32 currentFrame 
string message
")



(provide :kuavo_msgs/setMmCtrlFrame "696a444d2580aa682923215a4a34937c")


