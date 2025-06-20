;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::lejuClawState)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'lejuClawState (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::LEJUCLAWSTATE")
  (make-package "KUAVO_MSGS::LEJUCLAWSTATE"))

(in-package "ROS")
;;//! \htmlinclude lejuClawState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*KERROR*" (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(shadow '*KERROR* (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(defconstant kuavo_msgs::lejuClawState::*KERROR* -1)
(intern "*KUNKNOWN*" (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(shadow '*KUNKNOWN* (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(defconstant kuavo_msgs::lejuClawState::*KUNKNOWN* 0)
(intern "*KMOVING*" (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(shadow '*KMOVING* (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(defconstant kuavo_msgs::lejuClawState::*KMOVING* 1)
(intern "*KREACHED*" (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(shadow '*KREACHED* (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(defconstant kuavo_msgs::lejuClawState::*KREACHED* 2)
(intern "*KGRABBED*" (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(shadow '*KGRABBED* (find-package "KUAVO_MSGS::LEJUCLAWSTATE"))
(defconstant kuavo_msgs::lejuClawState::*KGRABBED* 3)

(defun kuavo_msgs::lejuClawState-to-symbol (const)
  (cond
        ((= const -1) 'kuavo_msgs::lejuClawState::*kError*)
        ((= const 0) 'kuavo_msgs::lejuClawState::*kUnknown*)
        ((= const 1) 'kuavo_msgs::lejuClawState::*kMoving*)
        ((= const 2) 'kuavo_msgs::lejuClawState::*kReached*)
        ((= const 3) 'kuavo_msgs::lejuClawState::*kGrabbed*)
        (t nil)))

(defclass kuavo_msgs::lejuClawState
  :super ros::object
  :slots (_header _state _data ))

(defmethod kuavo_msgs::lejuClawState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (make-array 0 :initial-element 0 :element-type :integer))
    ((:data __data) (instance kuavo_msgs::endEffectorData :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state __state)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
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
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8[] _state
    (* 1    (length _state)) 4
    ;; kuavo_msgs/endEffectorData _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (write-byte (elt _state i) s)
       )
     ;; kuavo_msgs/endEffectorData _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _state i) 127) (setf (elt _state i) (- (elt _state i) 256)))
     ))
   ;; kuavo_msgs/endEffectorData _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(setf (get kuavo_msgs::lejuClawState :md5sum-) "71c0eb8f4803a00f5667de51a2f70aac")
(setf (get kuavo_msgs::lejuClawState :datatype-) "kuavo_msgs/lejuClawState")
(setf (get kuavo_msgs::lejuClawState :definition-)
      "std_msgs/Header header

int8 kError = -1                
int8 kUnknown = 0              
int8 kMoving = 1              
int8 kReached = 2            
int8 kGrabbed = 3         

int8[] state # 0:; left; 1: right
kuavo_msgs/endEffectorData data
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: kuavo_msgs/endEffectorData
string[] name  
float64[] position
float64[] velocity  
float64[] effort

")



(provide :kuavo_msgs/lejuClawState "71c0eb8f4803a00f5667de51a2f70aac")


