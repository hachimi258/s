;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::yoloDetection)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'yoloDetection (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::YOLODETECTION")
  (make-package "KUAVO_MSGS::YOLODETECTION"))

(in-package "ROS")
;;//! \htmlinclude yoloDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kuavo_msgs::yoloDetection
  :super ros::object
  :slots (_header _data ))

(defmethod kuavo_msgs::yoloDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) ())
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
    ;; kuavo_msgs/yoloOutputData[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; kuavo_msgs/yoloOutputData[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; kuavo_msgs/yoloOutputData[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance kuavo_msgs::yoloOutputData :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kuavo_msgs::yoloDetection :md5sum-) "442f2c4723f0369e79b97b3e674354d8")
(setf (get kuavo_msgs::yoloDetection :datatype-) "kuavo_msgs/yoloDetection")
(setf (get kuavo_msgs::yoloDetection :definition-)
      "std_msgs/Header header
yoloOutputData[] data
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
MSG: kuavo_msgs/yoloOutputData
string class_name
int32 class_id
float32 confidence
float32 x_pos
float32 y_pos
float32 height
float32 width
")



(provide :kuavo_msgs/yoloDetection "442f2c4723f0369e79b97b3e674354d8")


