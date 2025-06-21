;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::dexhandTouchState)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'dexhandTouchState (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::DEXHANDTOUCHSTATE")
  (make-package "KUAVO_MSGS::DEXHANDTOUCHSTATE"))

(in-package "ROS")
;;//! \htmlinclude dexhandTouchState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kuavo_msgs::dexhandTouchState
  :super ros::object
  :slots (_header _left_hand _right_hand ))

(defmethod kuavo_msgs::dexhandTouchState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_hand __left_hand) (let (r) (dotimes (i 5) (push (instance kuavo_msgs::touchSensorStatus :init) r)) r))
    ((:right_hand __right_hand) (let (r) (dotimes (i 5) (push (instance kuavo_msgs::touchSensorStatus :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_hand __left_hand)
   (setq _right_hand __right_hand)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_hand
   (&rest __left_hand)
   (if (keywordp (car __left_hand))
       (send* _left_hand __left_hand)
     (progn
       (if __left_hand (setq _left_hand (car __left_hand)))
       _left_hand)))
  (:right_hand
   (&rest __right_hand)
   (if (keywordp (car __right_hand))
       (send* _right_hand __right_hand)
     (progn
       (if __right_hand (setq _right_hand (car __right_hand)))
       _right_hand)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; kuavo_msgs/touchSensorStatus[5] _left_hand
    (apply #'+ (send-all _left_hand :serialization-length))
    ;; kuavo_msgs/touchSensorStatus[5] _right_hand
    (apply #'+ (send-all _right_hand :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; kuavo_msgs/touchSensorStatus[5] _left_hand
     (dolist (elem _left_hand)
       (send elem :serialize s)
       )
     ;; kuavo_msgs/touchSensorStatus[5] _right_hand
     (dolist (elem _right_hand)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; kuavo_msgs/touchSensorStatus[5] _left_hand
   (dotimes (i 5)
     (send (elt _left_hand i) :deserialize buf ptr-) (incf ptr- (send (elt _left_hand i) :serialization-length))
     )
   ;; kuavo_msgs/touchSensorStatus[5] _right_hand
   (dotimes (i 5)
     (send (elt _right_hand i) :deserialize buf ptr-) (incf ptr- (send (elt _right_hand i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get kuavo_msgs::dexhandTouchState :md5sum-) "ce777577e1167705dca90d1f63037a05")
(setf (get kuavo_msgs::dexhandTouchState :datatype-) "kuavo_msgs/dexhandTouchState")
(setf (get kuavo_msgs::dexhandTouchState :definition-)
      "std_msgs/Header header
kuavo_msgs/touchSensorStatus[5] left_hand
kuavo_msgs/touchSensorStatus[5] right_hand

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
MSG: kuavo_msgs/touchSensorStatus
uint16 normal_force1  # 法向力1
uint16 normal_force2  # 法向力2
uint16 normal_force3  # 法向力3
uint16 tangential_force1  # 切向力1
uint16 tangential_force2  # 切向力2
uint16 tangential_force3  # 切向力3
uint16 tangential_direction1  # 切向力方向1
uint16 tangential_direction2  # 切向力方向2
uint16 tangential_direction3  # 切向力方向3
uint32 self_proximity1  # 自电容接近传感器1
uint32 self_proximity2  # 自电容接近传感器2
uint32 mutual_proximity  # 互电容接近传感器
uint16 status  # 传感器状态
")



(provide :kuavo_msgs/dexhandTouchState "ce777577e1167705dca90d1f63037a05")


