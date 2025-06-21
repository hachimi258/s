;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::handForceLevel)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'handForceLevel (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::HANDFORCELEVEL")
  (make-package "KUAVO_MSGS::HANDFORCELEVEL"))
(unless (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST")
  (make-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(unless (find-package "KUAVO_MSGS::HANDFORCELEVELRESPONSE")
  (make-package "KUAVO_MSGS::HANDFORCELEVELRESPONSE"))

(in-package "ROS")





(intern "*SMALL*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*SMALL* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*SMALL* 0)
(intern "*NORMAL*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*NORMAL* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*NORMAL* 1)
(intern "*FULL*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*FULL* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*FULL* 2)
(intern "*LEFT_HAND*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*LEFT_HAND* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*LEFT_HAND* 0)
(intern "*RIGHT_HAND*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*RIGHT_HAND* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*RIGHT_HAND* 1)
(intern "*BOTH_HANDS*" (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(shadow '*BOTH_HANDS* (find-package "KUAVO_MSGS::HANDFORCELEVELREQUEST"))
(defconstant kuavo_msgs::handForceLevelRequest::*BOTH_HANDS* 2)

(defun kuavo_msgs::handForceLevelRequest-to-symbol (const)
  (cond
        ((= const 0) 'kuavo_msgs::handForceLevelRequest::*SMALL*)
        ((= const 1) 'kuavo_msgs::handForceLevelRequest::*NORMAL*)
        ((= const 2) 'kuavo_msgs::handForceLevelRequest::*FULL*)
        ((= const 0) 'kuavo_msgs::handForceLevelRequest::*LEFT_HAND*)
        ((= const 1) 'kuavo_msgs::handForceLevelRequest::*RIGHT_HAND*)
        ((= const 2) 'kuavo_msgs::handForceLevelRequest::*BOTH_HANDS*)
        (t nil)))

(defclass kuavo_msgs::handForceLevelRequest
  :super ros::object
  :slots (_force_level _hand_side ))

(defmethod kuavo_msgs::handForceLevelRequest
  (:init
   (&key
    ((:force_level __force_level) 0)
    ((:hand_side __hand_side) 0)
    )
   (send-super :init)
   (setq _force_level (round __force_level))
   (setq _hand_side (round __hand_side))
   self)
  (:force_level
   (&optional __force_level)
   (if __force_level (setq _force_level __force_level)) _force_level)
  (:hand_side
   (&optional __hand_side)
   (if __hand_side (setq _hand_side __hand_side)) _hand_side)
  (:serialization-length
   ()
   (+
    ;; int8 _force_level
    1
    ;; int8 _hand_side
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _force_level
       (write-byte _force_level s)
     ;; int8 _hand_side
       (write-byte _hand_side s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _force_level
     (setq _force_level (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _force_level 127) (setq _force_level (- _force_level 256)))
   ;; int8 _hand_side
     (setq _hand_side (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _hand_side 127) (setq _hand_side (- _hand_side 256)))
   ;;
   self)
  )

(defclass kuavo_msgs::handForceLevelResponse
  :super ros::object
  :slots (_success _message _force_level ))

(defmethod kuavo_msgs::handForceLevelResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:force_level __force_level) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _force_level __force_level)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:force_level
   (&optional __force_level)
   (if __force_level (setq _force_level __force_level)) _force_level)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; int8[] _force_level
    (* 1    (length _force_level)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; int8[] _force_level
     (write-long (length _force_level) s)
     (dotimes (i (length _force_level))
       (write-byte (elt _force_level i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8[] _force_level
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _force_level (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _force_level i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _force_level i) 127) (setf (elt _force_level i) (- (elt _force_level i) 256)))
     ))
   ;;
   self)
  )

(defclass kuavo_msgs::handForceLevel
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::handForceLevel :md5sum-) "0f6c4fd291557ab445334f0487fbaf78")
(setf (get kuavo_msgs::handForceLevel :datatype-) "kuavo_msgs/handForceLevel")
(setf (get kuavo_msgs::handForceLevel :request) kuavo_msgs::handForceLevelRequest)
(setf (get kuavo_msgs::handForceLevel :response) kuavo_msgs::handForceLevelResponse)

(defmethod kuavo_msgs::handForceLevelRequest
  (:response () (instance kuavo_msgs::handForceLevelResponse :init)))

(setf (get kuavo_msgs::handForceLevelRequest :md5sum-) "0f6c4fd291557ab445334f0487fbaf78")
(setf (get kuavo_msgs::handForceLevelRequest :datatype-) "kuavo_msgs/handForceLevelRequest")
(setf (get kuavo_msgs::handForceLevelRequest :definition-)
      "# This service sets the force level for the Dexhand.
# It is used to control the force applied by the Dexhand.
#
# Request:
# int8 SMALL = 0          # Small force level.
# int8 NORMAL = 1         # Normal force level.
# int8 FULL = 2           # Full force level.
# int8 force_level        # The desired force level to set.
#
# Response:
# bool success            # Indicates whether the request was successful.
# string message          # A message indicating the result of the request.

int8 SMALL = 0
int8 NORMAL = 1
int8 FULL = 2

int8 LEFT_HAND=0
int8 RIGHT_HAND=1
int8 BOTH_HANDS=2

int8 force_level
int8 hand_side
---
bool success
string message
int8[] force_level
")

(setf (get kuavo_msgs::handForceLevelResponse :md5sum-) "0f6c4fd291557ab445334f0487fbaf78")
(setf (get kuavo_msgs::handForceLevelResponse :datatype-) "kuavo_msgs/handForceLevelResponse")
(setf (get kuavo_msgs::handForceLevelResponse :definition-)
      "# This service sets the force level for the Dexhand.
# It is used to control the force applied by the Dexhand.
#
# Request:
# int8 SMALL = 0          # Small force level.
# int8 NORMAL = 1         # Normal force level.
# int8 FULL = 2           # Full force level.
# int8 force_level        # The desired force level to set.
#
# Response:
# bool success            # Indicates whether the request was successful.
# string message          # A message indicating the result of the request.

int8 SMALL = 0
int8 NORMAL = 1
int8 FULL = 2

int8 LEFT_HAND=0
int8 RIGHT_HAND=1
int8 BOTH_HANDS=2

int8 force_level
int8 hand_side
---
bool success
string message
int8[] force_level
")



(provide :kuavo_msgs/handForceLevel "0f6c4fd291557ab445334f0487fbaf78")


