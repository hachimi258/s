;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::twoArmHandPose)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'twoArmHandPose (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::TWOARMHANDPOSE")
  (make-package "KUAVO_MSGS::TWOARMHANDPOSE"))

(in-package "ROS")
;;//! \htmlinclude twoArmHandPose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kuavo_msgs::twoArmHandPose
  :super ros::object
  :slots (_header _left_pose _right_pose ))

(defmethod kuavo_msgs::twoArmHandPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_pose __left_pose) (instance kuavo_msgs::armHandPose :init))
    ((:right_pose __right_pose) (instance kuavo_msgs::armHandPose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_pose __left_pose)
   (setq _right_pose __right_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_pose
   (&rest __left_pose)
   (if (keywordp (car __left_pose))
       (send* _left_pose __left_pose)
     (progn
       (if __left_pose (setq _left_pose (car __left_pose)))
       _left_pose)))
  (:right_pose
   (&rest __right_pose)
   (if (keywordp (car __right_pose))
       (send* _right_pose __right_pose)
     (progn
       (if __right_pose (setq _right_pose (car __right_pose)))
       _right_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; kuavo_msgs/armHandPose _left_pose
    (send _left_pose :serialization-length)
    ;; kuavo_msgs/armHandPose _right_pose
    (send _right_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; kuavo_msgs/armHandPose _left_pose
       (send _left_pose :serialize s)
     ;; kuavo_msgs/armHandPose _right_pose
       (send _right_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; kuavo_msgs/armHandPose _left_pose
     (send _left_pose :deserialize buf ptr-) (incf ptr- (send _left_pose :serialization-length))
   ;; kuavo_msgs/armHandPose _right_pose
     (send _right_pose :deserialize buf ptr-) (incf ptr- (send _right_pose :serialization-length))
   ;;
   self)
  )

(setf (get kuavo_msgs::twoArmHandPose :md5sum-) "5bdb1e027f430369b6f88e5e2f5d31ca")
(setf (get kuavo_msgs::twoArmHandPose :datatype-) "kuavo_msgs/twoArmHandPose")
(setf (get kuavo_msgs::twoArmHandPose :definition-)
      "Header header
armHandPose  left_pose
armHandPose  right_pose
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
MSG: kuavo_msgs/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

float64[7] joint_angles
")



(provide :kuavo_msgs/twoArmHandPose "5bdb1e027f430369b6f88e5e2f5d31ca")


