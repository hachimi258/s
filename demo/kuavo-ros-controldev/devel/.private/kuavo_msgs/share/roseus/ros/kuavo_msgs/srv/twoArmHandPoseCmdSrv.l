;; Auto-generated. Do not edit!


(when (boundp 'kuavo_msgs::twoArmHandPoseCmdSrv)
  (if (not (find-package "KUAVO_MSGS"))
    (make-package "KUAVO_MSGS"))
  (shadow 'twoArmHandPoseCmdSrv (find-package "KUAVO_MSGS")))
(unless (find-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRV")
  (make-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRV"))
(unless (find-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRVREQUEST")
  (make-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRVREQUEST"))
(unless (find-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRVRESPONSE")
  (make-package "KUAVO_MSGS::TWOARMHANDPOSECMDSRVRESPONSE"))

(in-package "ROS")





(defclass kuavo_msgs::twoArmHandPoseCmdSrvRequest
  :super ros::object
  :slots (_twoArmHandPoseCmdRequest ))

(defmethod kuavo_msgs::twoArmHandPoseCmdSrvRequest
  (:init
   (&key
    ((:twoArmHandPoseCmdRequest __twoArmHandPoseCmdRequest) (instance kuavo_msgs::twoArmHandPoseCmd :init))
    )
   (send-super :init)
   (setq _twoArmHandPoseCmdRequest __twoArmHandPoseCmdRequest)
   self)
  (:twoArmHandPoseCmdRequest
   (&rest __twoArmHandPoseCmdRequest)
   (if (keywordp (car __twoArmHandPoseCmdRequest))
       (send* _twoArmHandPoseCmdRequest __twoArmHandPoseCmdRequest)
     (progn
       (if __twoArmHandPoseCmdRequest (setq _twoArmHandPoseCmdRequest (car __twoArmHandPoseCmdRequest)))
       _twoArmHandPoseCmdRequest)))
  (:serialization-length
   ()
   (+
    ;; kuavo_msgs/twoArmHandPoseCmd _twoArmHandPoseCmdRequest
    (send _twoArmHandPoseCmdRequest :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kuavo_msgs/twoArmHandPoseCmd _twoArmHandPoseCmdRequest
       (send _twoArmHandPoseCmdRequest :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kuavo_msgs/twoArmHandPoseCmd _twoArmHandPoseCmdRequest
     (send _twoArmHandPoseCmdRequest :deserialize buf ptr-) (incf ptr- (send _twoArmHandPoseCmdRequest :serialization-length))
   ;;
   self)
  )

(defclass kuavo_msgs::twoArmHandPoseCmdSrvResponse
  :super ros::object
  :slots (_success _with_torso _q_arm _q_torso _time_cost _hand_poses ))

(defmethod kuavo_msgs::twoArmHandPoseCmdSrvResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:with_torso __with_torso) nil)
    ((:q_arm __q_arm) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:q_torso __q_torso) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:time_cost __time_cost) 0.0)
    ((:hand_poses __hand_poses) (instance kuavo_msgs::twoArmHandPose :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _with_torso __with_torso)
   (setq _q_arm __q_arm)
   (setq _q_torso __q_torso)
   (setq _time_cost (float __time_cost))
   (setq _hand_poses __hand_poses)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:with_torso
   (&optional (__with_torso :null))
   (if (not (eq __with_torso :null)) (setq _with_torso __with_torso)) _with_torso)
  (:q_arm
   (&optional __q_arm)
   (if __q_arm (setq _q_arm __q_arm)) _q_arm)
  (:q_torso
   (&optional __q_torso)
   (if __q_torso (setq _q_torso __q_torso)) _q_torso)
  (:time_cost
   (&optional __time_cost)
   (if __time_cost (setq _time_cost __time_cost)) _time_cost)
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
    ;; bool _with_torso
    1
    ;; float64[] _q_arm
    (* 8    (length _q_arm)) 4
    ;; float64[] _q_torso
    (* 8    (length _q_torso)) 4
    ;; float64 _time_cost
    8
    ;; kuavo_msgs/twoArmHandPose _hand_poses
    (send _hand_poses :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; bool _with_torso
       (if _with_torso (write-byte -1 s) (write-byte 0 s))
     ;; float64[] _q_arm
     (write-long (length _q_arm) s)
     (dotimes (i (length _q_arm))
       (sys::poke (elt _q_arm i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _q_torso
     (write-long (length _q_torso) s)
     (dotimes (i (length _q_torso))
       (sys::poke (elt _q_torso i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _time_cost
       (sys::poke _time_cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; kuavo_msgs/twoArmHandPose _hand_poses
       (send _hand_poses :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _with_torso
     (setq _with_torso (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[] _q_arm
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q_arm (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q_arm i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _q_torso
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q_torso (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q_torso i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _time_cost
     (setq _time_cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; kuavo_msgs/twoArmHandPose _hand_poses
     (send _hand_poses :deserialize buf ptr-) (incf ptr- (send _hand_poses :serialization-length))
   ;;
   self)
  )

(defclass kuavo_msgs::twoArmHandPoseCmdSrv
  :super ros::object
  :slots ())

(setf (get kuavo_msgs::twoArmHandPoseCmdSrv :md5sum-) "5c40c421651b828448c8e8f1a4ba5e3d")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrv :datatype-) "kuavo_msgs/twoArmHandPoseCmdSrv")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrv :request) kuavo_msgs::twoArmHandPoseCmdSrvRequest)
(setf (get kuavo_msgs::twoArmHandPoseCmdSrv :response) kuavo_msgs::twoArmHandPoseCmdSrvResponse)

(defmethod kuavo_msgs::twoArmHandPoseCmdSrvRequest
  (:response () (instance kuavo_msgs::twoArmHandPoseCmdSrvResponse :init)))

(setf (get kuavo_msgs::twoArmHandPoseCmdSrvRequest :md5sum-) "5c40c421651b828448c8e8f1a4ba5e3d")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrvRequest :datatype-) "kuavo_msgs/twoArmHandPoseCmdSrvRequest")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrvRequest :definition-)
      "twoArmHandPoseCmd twoArmHandPoseCmdRequest

================================================================================
MSG: kuavo_msgs/twoArmHandPoseCmd
twoArmHandPose  hand_poses
# params for the IK solver
bool use_custom_ik_param
bool joint_angles_as_q0
ikSolveParam ik_param
int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame   3  manipulation world frame 
================================================================================
MSG: kuavo_msgs/twoArmHandPose
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
MSG: kuavo_msgs/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

float64[7] joint_angles
================================================================================
MSG: kuavo_msgs/ikSolveParam
# snopt params
float64 major_optimality_tol
float64 major_feasibility_tol
float64 minor_feasibility_tol
float64 major_iterations_limit
# constraint and cost params
float64 oritation_constraint_tol
float64 pos_constraint_tol # work when pos_cost_weight > 0.0
---
bool success
bool with_torso
float64[] q_arm
float64[] q_torso

float64   time_cost # unit: ms
# ik result
twoArmHandPose  hand_poses

================================================================================
MSG: kuavo_msgs/twoArmHandPose
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
MSG: kuavo_msgs/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

")

(setf (get kuavo_msgs::twoArmHandPoseCmdSrvResponse :md5sum-) "5c40c421651b828448c8e8f1a4ba5e3d")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrvResponse :datatype-) "kuavo_msgs/twoArmHandPoseCmdSrvResponse")
(setf (get kuavo_msgs::twoArmHandPoseCmdSrvResponse :definition-)
      "twoArmHandPoseCmd twoArmHandPoseCmdRequest

================================================================================
MSG: kuavo_msgs/twoArmHandPoseCmd
twoArmHandPose  hand_poses
# params for the IK solver
bool use_custom_ik_param
bool joint_angles_as_q0
ikSolveParam ik_param
int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame   3  manipulation world frame 
================================================================================
MSG: kuavo_msgs/twoArmHandPose
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
MSG: kuavo_msgs/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

float64[7] joint_angles
================================================================================
MSG: kuavo_msgs/ikSolveParam
# snopt params
float64 major_optimality_tol
float64 major_feasibility_tol
float64 minor_feasibility_tol
float64 major_iterations_limit
# constraint and cost params
float64 oritation_constraint_tol
float64 pos_constraint_tol # work when pos_cost_weight > 0.0
---
bool success
bool with_torso
float64[] q_arm
float64[] q_torso

float64   time_cost # unit: ms
# ik result
twoArmHandPose  hand_poses

================================================================================
MSG: kuavo_msgs/twoArmHandPose
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
MSG: kuavo_msgs/armHandPose
float64[3] pos_xyz
float64[4] quat_xyzw

float64[3] elbow_pos_xyz

")



(provide :kuavo_msgs/twoArmHandPoseCmdSrv "5c40c421651b828448c8e8f1a4ba5e3d")


