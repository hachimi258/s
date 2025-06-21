; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-msg)


;//! \htmlinclude twoArmHandPoseCmd.msg.html

(cl:defclass <twoArmHandPoseCmd> (roslisp-msg-protocol:ros-message)
  ((hand_poses
    :reader hand_poses
    :initarg :hand_poses
    :type kuavo_msgs-msg:twoArmHandPose
    :initform (cl:make-instance 'kuavo_msgs-msg:twoArmHandPose))
   (use_custom_ik_param
    :reader use_custom_ik_param
    :initarg :use_custom_ik_param
    :type cl:boolean
    :initform cl:nil)
   (joint_angles_as_q0
    :reader joint_angles_as_q0
    :initarg :joint_angles_as_q0
    :type cl:boolean
    :initform cl:nil)
   (ik_param
    :reader ik_param
    :initarg :ik_param
    :type kuavo_msgs-msg:ikSolveParam
    :initform (cl:make-instance 'kuavo_msgs-msg:ikSolveParam))
   (frame
    :reader frame
    :initarg :frame
    :type cl:integer
    :initform 0))
)

(cl:defclass twoArmHandPoseCmd (<twoArmHandPoseCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <twoArmHandPoseCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'twoArmHandPoseCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-msg:<twoArmHandPoseCmd> is deprecated: use kuavo_msgs-msg:twoArmHandPoseCmd instead.")))

(cl:ensure-generic-function 'hand_poses-val :lambda-list '(m))
(cl:defmethod hand_poses-val ((m <twoArmHandPoseCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:hand_poses-val is deprecated.  Use kuavo_msgs-msg:hand_poses instead.")
  (hand_poses m))

(cl:ensure-generic-function 'use_custom_ik_param-val :lambda-list '(m))
(cl:defmethod use_custom_ik_param-val ((m <twoArmHandPoseCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:use_custom_ik_param-val is deprecated.  Use kuavo_msgs-msg:use_custom_ik_param instead.")
  (use_custom_ik_param m))

(cl:ensure-generic-function 'joint_angles_as_q0-val :lambda-list '(m))
(cl:defmethod joint_angles_as_q0-val ((m <twoArmHandPoseCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:joint_angles_as_q0-val is deprecated.  Use kuavo_msgs-msg:joint_angles_as_q0 instead.")
  (joint_angles_as_q0 m))

(cl:ensure-generic-function 'ik_param-val :lambda-list '(m))
(cl:defmethod ik_param-val ((m <twoArmHandPoseCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:ik_param-val is deprecated.  Use kuavo_msgs-msg:ik_param instead.")
  (ik_param m))

(cl:ensure-generic-function 'frame-val :lambda-list '(m))
(cl:defmethod frame-val ((m <twoArmHandPoseCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-msg:frame-val is deprecated.  Use kuavo_msgs-msg:frame instead.")
  (frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <twoArmHandPoseCmd>) ostream)
  "Serializes a message object of type '<twoArmHandPoseCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hand_poses) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_custom_ik_param) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_angles_as_q0) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ik_param) ostream)
  (cl:let* ((signed (cl:slot-value msg 'frame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <twoArmHandPoseCmd>) istream)
  "Deserializes a message object of type '<twoArmHandPoseCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hand_poses) istream)
    (cl:setf (cl:slot-value msg 'use_custom_ik_param) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_angles_as_q0) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ik_param) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<twoArmHandPoseCmd>)))
  "Returns string type for a message object of type '<twoArmHandPoseCmd>"
  "kuavo_msgs/twoArmHandPoseCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'twoArmHandPoseCmd)))
  "Returns string type for a message object of type 'twoArmHandPoseCmd"
  "kuavo_msgs/twoArmHandPoseCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<twoArmHandPoseCmd>)))
  "Returns md5sum for a message object of type '<twoArmHandPoseCmd>"
  "cd5f0a3dc4154eb55aff1c874e2dc81e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'twoArmHandPoseCmd)))
  "Returns md5sum for a message object of type 'twoArmHandPoseCmd"
  "cd5f0a3dc4154eb55aff1c874e2dc81e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<twoArmHandPoseCmd>)))
  "Returns full string definition for message of type '<twoArmHandPoseCmd>"
  (cl:format cl:nil "twoArmHandPose  hand_poses~%# params for the IK solver~%bool use_custom_ik_param~%bool joint_angles_as_q0~%ikSolveParam ik_param~%int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame   3  manipulation world frame ~%================================================================================~%MSG: kuavo_msgs/twoArmHandPose~%Header header~%armHandPose  left_pose~%armHandPose  right_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/armHandPose~%float64[3] pos_xyz~%float64[4] quat_xyzw~%~%float64[3] elbow_pos_xyz~%~%float64[7] joint_angles~%================================================================================~%MSG: kuavo_msgs/ikSolveParam~%# snopt params~%float64 major_optimality_tol~%float64 major_feasibility_tol~%float64 minor_feasibility_tol~%float64 major_iterations_limit~%# constraint and cost params~%float64 oritation_constraint_tol~%float64 pos_constraint_tol # work when pos_cost_weight > 0.0~%float64 pos_cost_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'twoArmHandPoseCmd)))
  "Returns full string definition for message of type 'twoArmHandPoseCmd"
  (cl:format cl:nil "twoArmHandPose  hand_poses~%# params for the IK solver~%bool use_custom_ik_param~%bool joint_angles_as_q0~%ikSolveParam ik_param~%int32 frame # 0 keep current frame  1 world frame (based on odom)  2  local frame   3  manipulation world frame ~%================================================================================~%MSG: kuavo_msgs/twoArmHandPose~%Header header~%armHandPose  left_pose~%armHandPose  right_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kuavo_msgs/armHandPose~%float64[3] pos_xyz~%float64[4] quat_xyzw~%~%float64[3] elbow_pos_xyz~%~%float64[7] joint_angles~%================================================================================~%MSG: kuavo_msgs/ikSolveParam~%# snopt params~%float64 major_optimality_tol~%float64 major_feasibility_tol~%float64 minor_feasibility_tol~%float64 major_iterations_limit~%# constraint and cost params~%float64 oritation_constraint_tol~%float64 pos_constraint_tol # work when pos_cost_weight > 0.0~%float64 pos_cost_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <twoArmHandPoseCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hand_poses))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ik_param))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <twoArmHandPoseCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'twoArmHandPoseCmd
    (cl:cons ':hand_poses (hand_poses msg))
    (cl:cons ':use_custom_ik_param (use_custom_ik_param msg))
    (cl:cons ':joint_angles_as_q0 (joint_angles_as_q0 msg))
    (cl:cons ':ik_param (ik_param msg))
    (cl:cons ':frame (frame msg))
))
