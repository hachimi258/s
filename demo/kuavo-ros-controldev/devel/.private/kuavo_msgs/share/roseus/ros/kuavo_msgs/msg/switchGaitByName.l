;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::switchGaitByName)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'switchGaitByName (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::SWITCHGAITBYNAME")
  (make-package "KUAVO_MSGS::SWITCHGAITBYNAME"))

(in-package "ROS")
;;//! \htmlinclude switchGaitByName.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kuavo_msgs::switchGaitByName
  :super ros::object
  :slots (_header _gait_name ))

(defmethod kuavo_msgs::switchGaitByName
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:gait_name __gait_name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _gait_name (string __gait_name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:gait_name
   (&optional __gait_name)
   (if __gait_name (setq _gait_name __gait_name)) _gait_name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _gait_name
    4 (length _gait_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _gait_name
       (write-long (length _gait_name) s) (princ _gait_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _gait_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gait_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kuavo_msgs::switchGaitByName :md5sum-) "f98f610ddbe99ec14bb5241e5221ca96")
(setf (get kuavo_msgs::switchGaitByName :datatype-) "kuavo_msgs/switchGaitByName")
(setf (get kuavo_msgs::switchGaitByName :definition-)
      "std_msgs/Header header
string  gait_name

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



(provide :kuavo_msgs/switchGaitByName "f98f610ddbe99ec14bb5241e5221ca96")


