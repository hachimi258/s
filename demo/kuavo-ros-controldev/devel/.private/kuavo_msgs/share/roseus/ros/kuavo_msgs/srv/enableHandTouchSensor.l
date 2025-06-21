;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::enableHandTouchSensor)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'enableHandTouchSensor (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSOR")
  (make-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSOR"))
(unless (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST")
  (make-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(unless (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORRESPONSE")
  (make-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORRESPONSE"))

(in-package "ROS")





(intern "*THUMB_SENSOR*" (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(shadow '*THUMB_SENSOR* (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(defconstant kuavo_msgs::enableHandTouchSensorRequest::*THUMB_SENSOR* 1)
(intern "*INDEX_SENSOR*" (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(shadow '*INDEX_SENSOR* (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(defconstant kuavo_msgs::enableHandTouchSensorRequest::*INDEX_SENSOR* 2)
(intern "*MIDDLE_SENSOR*" (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(shadow '*MIDDLE_SENSOR* (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(defconstant kuavo_msgs::enableHandTouchSensorRequest::*MIDDLE_SENSOR* 4)
(intern "*RING_SENSOR*" (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(shadow '*RING_SENSOR* (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(defconstant kuavo_msgs::enableHandTouchSensorRequest::*RING_SENSOR* 8)
(intern "*PINKY_SENSOR*" (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(shadow '*PINKY_SENSOR* (find-package "KUAVO_MSGS::ENABLEHANDTOUCHSENSORREQUEST"))
(defconstant kuavo_msgs::enableHandTouchSensorRequest::*PINKY_SENSOR* 16)

(defun kuavo_msgs::enableHandTouchSensorRequest-to-symbol (const)
  (cond
        ((= const 1) 'kuavo_msgs::enableHandTouchSensorRequest::*THUMB_SENSOR*)
        ((= const 2) 'kuavo_msgs::enableHandTouchSensorRequest::*INDEX_SENSOR*)
        ((= const 4) 'kuavo_msgs::enableHandTouchSensorRequest::*MIDDLE_SENSOR*)
        ((= const 8) 'kuavo_msgs::enableHandTouchSensorRequest::*RING_SENSOR*)
        ((= const 16) 'kuavo_msgs::enableHandTouchSensorRequest::*PINKY_SENSOR*)
        (t nil)))

(defclass kuavo_msgs::enableHandTouchSensorRequest
  :super ros::object
  :slots (_mask ))

(defmethod kuavo_msgs::enableHandTouchSensorRequest
  (:init
   (&key
    ((:mask __mask) 0)
    )
   (send-super :init)
   (setq _mask (round __mask))
   self)
  (:mask
   (&optional __mask)
   (if __mask (setq _mask __mask)) _mask)
  (:serialization-length
   ()
   (+
    ;; uint8 _mask
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mask
       (write-byte _mask s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mask
     (setq _mask (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass kuavo_msgs::enableHandTouchSensorResponse
  :super ros::object
  :slots (_success _message ))

(defmethod kuavo_msgs::enableHandTouchSensorResponse
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

(defclass kuavo_msgs::enableHandTouchSensor
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::enableHandTouchSensor :md5sum-) "710f3d70d245856e41b01f7ffef21580")
(setf (get kuavo_msgs::enableHandTouchSensor :datatype-) "kuavo_msgs/enableHandTouchSensor")
(setf (get kuavo_msgs::enableHandTouchSensor :request) kuavo_msgs::enableHandTouchSensorRequest)
(setf (get kuavo_msgs::enableHandTouchSensor :response) kuavo_msgs::enableHandTouchSensorResponse)

(defmethod kuavo_msgs::enableHandTouchSensorRequest
  (:response () (instance kuavo_msgs::enableHandTouchSensorResponse :init)))

(setf (get kuavo_msgs::enableHandTouchSensorRequest :md5sum-) "710f3d70d245856e41b01f7ffef21580")
(setf (get kuavo_msgs::enableHandTouchSensorRequest :datatype-) "kuavo_msgs/enableHandTouchSensorRequest")
(setf (get kuavo_msgs::enableHandTouchSensorRequest :definition-)
      "# Bit masks for enabling individual touch sensors on the robotic hand
# Each sensor can be enabled by setting the corresponding bit in the mask

# Examples: 0b00000011 enables thumb and index sensors, 0b00000000 disables all sensors
#  ``` 
#    mask_value = THUMB_SENSOR | INDEX_SENSOR
#    req = enableHandTouchSensorRequest()
#    req.mask = mask_value
#  ``` 

# Thumb finger touch sensor (bit 0)
uint8 THUMB_SENSOR = 1

# Index finger touch sensor (bit 1)
uint8 INDEX_SENSOR = 2

# Middle finger touch sensor (bit 2)
uint8 MIDDLE_SENSOR = 4

# Ring finger touch sensor (bit 3)
uint8 RING_SENSOR = 8

# Pinky finger touch sensor (bit 4)
uint8 PINKY_SENSOR = 16

# Bitmask indicating which sensors to enable
# Multiple sensors can be enabled by combining masks with bitwise OR
uint8 mask
---
# Whether the operation was successful
bool success

# Additional status or error message
string message
")

(setf (get kuavo_msgs::enableHandTouchSensorResponse :md5sum-) "710f3d70d245856e41b01f7ffef21580")
(setf (get kuavo_msgs::enableHandTouchSensorResponse :datatype-) "kuavo_msgs/enableHandTouchSensorResponse")
(setf (get kuavo_msgs::enableHandTouchSensorResponse :definition-)
      "# Bit masks for enabling individual touch sensors on the robotic hand
# Each sensor can be enabled by setting the corresponding bit in the mask

# Examples: 0b00000011 enables thumb and index sensors, 0b00000000 disables all sensors
#  ``` 
#    mask_value = THUMB_SENSOR | INDEX_SENSOR
#    req = enableHandTouchSensorRequest()
#    req.mask = mask_value
#  ``` 

# Thumb finger touch sensor (bit 0)
uint8 THUMB_SENSOR = 1

# Index finger touch sensor (bit 1)
uint8 INDEX_SENSOR = 2

# Middle finger touch sensor (bit 2)
uint8 MIDDLE_SENSOR = 4

# Ring finger touch sensor (bit 3)
uint8 RING_SENSOR = 8

# Pinky finger touch sensor (bit 4)
uint8 PINKY_SENSOR = 16

# Bitmask indicating which sensors to enable
# Multiple sensors can be enabled by combining masks with bitwise OR
uint8 mask
---
# Whether the operation was successful
bool success

# Additional status or error message
string message
")



(provide :kuavo_msgs/enableHandTouchSensor "710f3d70d245856e41b01f7ffef21580")


