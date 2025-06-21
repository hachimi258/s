;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::dexhandCommand)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'dexhandCommand (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::DEXHANDCOMMAND")
  (make-package "KUAVO_MSGS::DEXHANDCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude dexhandCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*POSITION_CONTROL*" (find-package "KUAVO_MSGS::DEXHANDCOMMAND"))
(shadow '*POSITION_CONTROL* (find-package "KUAVO_MSGS::DEXHANDCOMMAND"))
(defconstant kuavo_msgs::dexhandCommand::*POSITION_CONTROL* 0)
(intern "*VELOCITY_CONTROL*" (find-package "KUAVO_MSGS::DEXHANDCOMMAND"))
(shadow '*VELOCITY_CONTROL* (find-package "KUAVO_MSGS::DEXHANDCOMMAND"))
(defconstant kuavo_msgs::dexhandCommand::*VELOCITY_CONTROL* 1)

(defun kuavo_msgs::dexhandCommand-to-symbol (const)
  (cond
        ((= const 0) 'kuavo_msgs::dexhandCommand::*POSITION_CONTROL*)
        ((= const 1) 'kuavo_msgs::dexhandCommand::*VELOCITY_CONTROL*)
        (t nil)))

(defclass kuavo_msgs::dexhandCommand
  :super ros::object
  :slots (_header _control_mode _data ))

(defmethod kuavo_msgs::dexhandCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:control_mode __control_mode) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _control_mode (round __control_mode))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _control_mode
    1
    ;; int16[] _data
    (* 2    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _control_mode
       (write-byte _control_mode s)
     ;; int16[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-word (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _control_mode 127) (setq _control_mode (- _control_mode 256)))
   ;; int16[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::dexhandCommand :md5sum-) "ab54699609aa0f9682d32ee67eae87dc")
(setf (get kuavo_msgs::dexhandCommand :datatype-) "kuavo_msgs/dexhandCommand")
(setf (get kuavo_msgs::dexhandCommand :definition-)
      "# Control modes
int8 POSITION_CONTROL = 0  # Position control mode
int8 VELOCITY_CONTROL = 1  # Velocity control mode

# Message header
std_msgs/Header header

# Control mode to be used
int8 control_mode

# Data array
# 数据数组，单手时长度必须为6，双手长度必须为12
# - 位置控制模式下，每个元素的数据的范围为[0, 100], 0 为完全打开，100 为完全关闭
# - 速度控制模式下，每个元素的数据的范围为[-100, 100] 负数表示打开，正数表示关闭
int16[] data
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

")



(provide :kuavo_msgs/dexhandCommand "ab54699609aa0f9682d32ee67eae87dc")


