; Auto-generated. Do not edit!


(cl:in-package kuavo_msgs-srv)


;//! \htmlinclude footPoseTargetTrajectoriesSrv-request.msg.html

(cl:defclass <footPoseTargetTrajectoriesSrv-request> (roslisp-msg-protocol:ros-message)
  ((foot_pose_target_trajectories
    :reader foot_pose_target_trajectories
    :initarg :foot_pose_target_trajectories
    :type kuavo_msgs-msg:footPoseTargetTrajectories
    :initform (cl:make-instance 'kuavo_msgs-msg:footPoseTargetTrajectories)))
)

(cl:defclass footPoseTargetTrajectoriesSrv-request (<footPoseTargetTrajectoriesSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <footPoseTargetTrajectoriesSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'footPoseTargetTrajectoriesSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<footPoseTargetTrajectoriesSrv-request> is deprecated: use kuavo_msgs-srv:footPoseTargetTrajectoriesSrv-request instead.")))

(cl:ensure-generic-function 'foot_pose_target_trajectories-val :lambda-list '(m))
(cl:defmethod foot_pose_target_trajectories-val ((m <footPoseTargetTrajectoriesSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:foot_pose_target_trajectories-val is deprecated.  Use kuavo_msgs-srv:foot_pose_target_trajectories instead.")
  (foot_pose_target_trajectories m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <footPoseTargetTrajectoriesSrv-request>) ostream)
  "Serializes a message object of type '<footPoseTargetTrajectoriesSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'foot_pose_target_trajectories) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <footPoseTargetTrajectoriesSrv-request>) istream)
  "Deserializes a message object of type '<footPoseTargetTrajectoriesSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'foot_pose_target_trajectories) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<footPoseTargetTrajectoriesSrv-request>)))
  "Returns string type for a service object of type '<footPoseTargetTrajectoriesSrv-request>"
  "kuavo_msgs/footPoseTargetTrajectoriesSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'footPoseTargetTrajectoriesSrv-request)))
  "Returns string type for a service object of type 'footPoseTargetTrajectoriesSrv-request"
  "kuavo_msgs/footPoseTargetTrajectoriesSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<footPoseTargetTrajectoriesSrv-request>)))
  "Returns md5sum for a message object of type '<footPoseTargetTrajectoriesSrv-request>"
  "86a3a0586a4bc1d73a3be321ecdc3a15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'footPoseTargetTrajectoriesSrv-request)))
  "Returns md5sum for a message object of type 'footPoseTargetTrajectoriesSrv-request"
  "86a3a0586a4bc1d73a3be321ecdc3a15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<footPoseTargetTrajectoriesSrv-request>)))
  "Returns full string definition for message of type '<footPoseTargetTrajectoriesSrv-request>"
  (cl:format cl:nil "footPoseTargetTrajectories foot_pose_target_trajectories~%~%================================================================================~%MSG: kuavo_msgs/footPoseTargetTrajectories~%float64[]    timeTrajectory~%int32[]      footIndexTrajectory~%footPose[]   footPoseTrajectory~%footPoses[]  additionalFootPoseTrajectory  # 可选字段，用于存储额外的轨迹点规划值~%~%================================================================================~%MSG: kuavo_msgs/footPose~%float64[4] footPose # x, y, z, yaw~%float64[4] torsoPose # x, y, z, yaw~%~%================================================================================~%MSG: kuavo_msgs/footPoses~%footPose[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'footPoseTargetTrajectoriesSrv-request)))
  "Returns full string definition for message of type 'footPoseTargetTrajectoriesSrv-request"
  (cl:format cl:nil "footPoseTargetTrajectories foot_pose_target_trajectories~%~%================================================================================~%MSG: kuavo_msgs/footPoseTargetTrajectories~%float64[]    timeTrajectory~%int32[]      footIndexTrajectory~%footPose[]   footPoseTrajectory~%footPoses[]  additionalFootPoseTrajectory  # 可选字段，用于存储额外的轨迹点规划值~%~%================================================================================~%MSG: kuavo_msgs/footPose~%float64[4] footPose # x, y, z, yaw~%float64[4] torsoPose # x, y, z, yaw~%~%================================================================================~%MSG: kuavo_msgs/footPoses~%footPose[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <footPoseTargetTrajectoriesSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'foot_pose_target_trajectories))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <footPoseTargetTrajectoriesSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'footPoseTargetTrajectoriesSrv-request
    (cl:cons ':foot_pose_target_trajectories (foot_pose_target_trajectories msg))
))
;//! \htmlinclude footPoseTargetTrajectoriesSrv-response.msg.html

(cl:defclass <footPoseTargetTrajectoriesSrv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass footPoseTargetTrajectoriesSrv-response (<footPoseTargetTrajectoriesSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <footPoseTargetTrajectoriesSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'footPoseTargetTrajectoriesSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kuavo_msgs-srv:<footPoseTargetTrajectoriesSrv-response> is deprecated: use kuavo_msgs-srv:footPoseTargetTrajectoriesSrv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <footPoseTargetTrajectoriesSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kuavo_msgs-srv:success-val is deprecated.  Use kuavo_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <footPoseTargetTrajectoriesSrv-response>) ostream)
  "Serializes a message object of type '<footPoseTargetTrajectoriesSrv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <footPoseTargetTrajectoriesSrv-response>) istream)
  "Deserializes a message object of type '<footPoseTargetTrajectoriesSrv-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<footPoseTargetTrajectoriesSrv-response>)))
  "Returns string type for a service object of type '<footPoseTargetTrajectoriesSrv-response>"
  "kuavo_msgs/footPoseTargetTrajectoriesSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'footPoseTargetTrajectoriesSrv-response)))
  "Returns string type for a service object of type 'footPoseTargetTrajectoriesSrv-response"
  "kuavo_msgs/footPoseTargetTrajectoriesSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<footPoseTargetTrajectoriesSrv-response>)))
  "Returns md5sum for a message object of type '<footPoseTargetTrajectoriesSrv-response>"
  "86a3a0586a4bc1d73a3be321ecdc3a15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'footPoseTargetTrajectoriesSrv-response)))
  "Returns md5sum for a message object of type 'footPoseTargetTrajectoriesSrv-response"
  "86a3a0586a4bc1d73a3be321ecdc3a15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<footPoseTargetTrajectoriesSrv-response>)))
  "Returns full string definition for message of type '<footPoseTargetTrajectoriesSrv-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'footPoseTargetTrajectoriesSrv-response)))
  "Returns full string definition for message of type 'footPoseTargetTrajectoriesSrv-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <footPoseTargetTrajectoriesSrv-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <footPoseTargetTrajectoriesSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'footPoseTargetTrajectoriesSrv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'footPoseTargetTrajectoriesSrv)))
  'footPoseTargetTrajectoriesSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'footPoseTargetTrajectoriesSrv)))
  'footPoseTargetTrajectoriesSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'footPoseTargetTrajectoriesSrv)))
  "Returns string type for a service object of type '<footPoseTargetTrajectoriesSrv>"
  "kuavo_msgs/footPoseTargetTrajectoriesSrv")