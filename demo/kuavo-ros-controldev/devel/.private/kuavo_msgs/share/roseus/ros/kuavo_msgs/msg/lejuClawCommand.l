;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::lejuClawCommand)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'lejuClawCommand (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::LEJUCLAWCOMMAND")
  (make-package "KUAVO_MSGS::LEJUCLAWCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude lejuClawCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kuavo_msgs::lejuClawCommand
  :super ros::object
  :slots (_header _data ))

(defmethod kuavo_msgs::lejuClawCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (instance kuavo_msgs::endEffectorData :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; kuavo_msgs/endEffectorData _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; kuavo_msgs/endEffectorData _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; kuavo_msgs/endEffectorData _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(setf (get kuavo_msgs::lejuClawCommand :md5sum-) "57c40e80f90e7a289ae0b2488552e043")
(setf (get kuavo_msgs::lejuClawCommand :datatype-) "kuavo_msgs/lejuClawCommand")
(setf (get kuavo_msgs::lejuClawCommand :definition-)
      "# kuavo_msgs/endEffectorData:
# string[] name  
# float64[] position
# float64[] velocity  
# float64[] effort
# 
# ** For the Topic Notes **
# 
# name     : 'left_claw' , 'right_claw'
# position : 0 ~ 100, the percentage of the claw's opening angle
#            0: closed, 100: open   
# velocity : 0 ~ 100, if size is 0, will use default `50.0`.
# effort   : torque/current, better 1A ~ 2A, if size is 0, will use default `1.0`.
# 
# ** Example **
# data:
#   - name: ['left_claw', 'right_claw']
#     position: [20.0, 20.0]
#     velocity: [50.0, 50.0]
#     effort: [1.0, 1.0]

std_msgs/Header header
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



(provide :kuavo_msgs/lejuClawCommand "57c40e80f90e7a289ae0b2488552e043")


