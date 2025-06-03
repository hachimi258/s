;; Auto-generated. Do not edit!


(when (boundp 'motion_capture_ik::fkSrv)
  (if (not (find-package "MOTION_CAPTURE_IK"))
    (make-package "MOTION_CAPTURE_IK"))
  (shadow 'fkSrv (find-package "MOTION_CAPTURE_IK")))
(unless (find-package "MOTION_CAPTURE_IK::FKSRV")
  (make-package "MOTION_CAPTURE_IK::FKSRV"))
(unless (find-package "MOTION_CAPTURE_IK::FKSRVREQUEST")
  (make-package "MOTION_CAPTURE_IK::FKSRVREQUEST"))
(unless (find-package "MOTION_CAPTURE_IK::FKSRVRESPONSE")
  (make-package "MOTION_CAPTURE_IK::FKSRVRESPONSE"))

(in-package "ROS")





(defclass motion_capture_ik::fkSrvRequest
  :super ros::object
  :slots (_q ))

(defmethod motion_capture_ik::fkSrvRequest
  (:init
   (&key
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:serialization-length
   ()
   (+
    ;; float64[] _q
    (* 8    (length _q)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _q
     (write-long (length _q) s)
     (dotimes (i (length _q))
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass motion_capture_ik::fkSrvResponse
  :super ros::object
  :slots (_success _hand_poses ))

(defmethod motion_capture_ik::fkSrvResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:hand_poses __hand_poses) (instance motion_capture_ik::twoArmHandPose :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _hand_poses __hand_poses)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:hand_poses
   (&rest __hand_poses)
   (if (keywordp (car __hand_poses))
       (send* _hand_poses __hand_poses)
     (progn
       (if __hand_poses (setq _hand_poses (car __hand_poses)))
       _hand_poses)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; motion_capture_ik/twoArmHandPose _hand_poses
    (send _hand_poses :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; motion_capture_ik/twoArmHandPose _hand_poses
       (send _hand_poses :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; motion_capture_ik/twoArmHandPose _hand_poses
     (send _hand_poses :deserialize buf ptr-) (incf ptr- (send _hand_poses :serialization-length))
   ;;
   self)
  )

(defclass motion_capture_ik::fkSrv
  :super ros::object
  :slots ())

(setf (get motion_capture_ik::fkSrv :md5sum-) "b89cc987a02b6d1c2a1588d5659bf064")
(setf (get motion_capture_ik::fkSrv :datatype-) "motion_capture_ik/fkSrv")
(setf (get motion_capture_ik::fkSrv :request) motion_capture_ik::fkSrvRequest)
(setf (get motion_capture_ik::fkSrv :response) motion_capture_ik::fkSrvResponse)

(defmethod motion_capture_ik::fkSrvRequest
  (:response () (instance motion_capture_ik::fkSrvResponse :init)))

(setf (get motion_capture_ik::fkSrvRequest :md5sum-) "b89cc987a02b6d1c2a1588d5659bf064")
(setf (get motion_capture_ik::fkSrvRequest :datatype-) "motion_capture_ik/fkSrvRequest")
(setf (get motion_capture_ik::fkSrvRequest :definition-)
      "float64[] q # 广义关节角度，如果是虚拟关节ik，则前4维度为躯干虚拟关节的角度，后14维度为手臂关节的角度
---
bool success
twoArmHandPose  hand_poses

================================================================================
MSG: motion_capture_ik/twoArmHandPose
Header header
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
MSG: motion_capture_ik/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

")

(setf (get motion_capture_ik::fkSrvResponse :md5sum-) "b89cc987a02b6d1c2a1588d5659bf064")
(setf (get motion_capture_ik::fkSrvResponse :datatype-) "motion_capture_ik/fkSrvResponse")
(setf (get motion_capture_ik::fkSrvResponse :definition-)
      "float64[] q # 广义关节角度，如果是虚拟关节ik，则前4维度为躯干虚拟关节的角度，后14维度为手臂关节的角度
---
bool success
twoArmHandPose  hand_poses

================================================================================
MSG: motion_capture_ik/twoArmHandPose
Header header
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
MSG: motion_capture_ik/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

")



(provide :motion_capture_ik/fkSrv "b89cc987a02b6d1c2a1588d5659bf064")


