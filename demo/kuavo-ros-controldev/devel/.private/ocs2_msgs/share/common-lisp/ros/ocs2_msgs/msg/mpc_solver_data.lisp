; Auto-generated. Do not edit!


(cl:in-package ocs2_msgs-msg)


;//! \htmlinclude mpc_solver_data.msg.html

(cl:defclass <mpc_solver_data> (roslisp-msg-protocol:ros-message)
  ((initTime
    :reader initTime
    :initarg :initTime
    :type cl:float
    :initform 0.0)
   (initState
    :reader initState
    :initarg :initState
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (finalTime
    :reader finalTime
    :initarg :finalTime
    :type cl:float
    :initform 0.0)
   (modeSchedule
    :reader modeSchedule
    :initarg :modeSchedule
    :type ocs2_msgs-msg:mode_schedule
    :initform (cl:make-instance 'ocs2_msgs-msg:mode_schedule))
   (targetTrajectories
    :reader targetTrajectories
    :initarg :targetTrajectories
    :type ocs2_msgs-msg:mpc_target_trajectories
    :initform (cl:make-instance 'ocs2_msgs-msg:mpc_target_trajectories))
   (mpc_flattened_controller
    :reader mpc_flattened_controller
    :initarg :mpc_flattened_controller
    :type ocs2_msgs-msg:mpc_flattened_controller
    :initform (cl:make-instance 'ocs2_msgs-msg:mpc_flattened_controller))
   (swingPlannerMultipliers
    :reader swingPlannerMultipliers
    :initarg :swingPlannerMultipliers
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass mpc_solver_data (<mpc_solver_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mpc_solver_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mpc_solver_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ocs2_msgs-msg:<mpc_solver_data> is deprecated: use ocs2_msgs-msg:mpc_solver_data instead.")))

(cl:ensure-generic-function 'initTime-val :lambda-list '(m))
(cl:defmethod initTime-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:initTime-val is deprecated.  Use ocs2_msgs-msg:initTime instead.")
  (initTime m))

(cl:ensure-generic-function 'initState-val :lambda-list '(m))
(cl:defmethod initState-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:initState-val is deprecated.  Use ocs2_msgs-msg:initState instead.")
  (initState m))

(cl:ensure-generic-function 'finalTime-val :lambda-list '(m))
(cl:defmethod finalTime-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:finalTime-val is deprecated.  Use ocs2_msgs-msg:finalTime instead.")
  (finalTime m))

(cl:ensure-generic-function 'modeSchedule-val :lambda-list '(m))
(cl:defmethod modeSchedule-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:modeSchedule-val is deprecated.  Use ocs2_msgs-msg:modeSchedule instead.")
  (modeSchedule m))

(cl:ensure-generic-function 'targetTrajectories-val :lambda-list '(m))
(cl:defmethod targetTrajectories-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:targetTrajectories-val is deprecated.  Use ocs2_msgs-msg:targetTrajectories instead.")
  (targetTrajectories m))

(cl:ensure-generic-function 'mpc_flattened_controller-val :lambda-list '(m))
(cl:defmethod mpc_flattened_controller-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:mpc_flattened_controller-val is deprecated.  Use ocs2_msgs-msg:mpc_flattened_controller instead.")
  (mpc_flattened_controller m))

(cl:ensure-generic-function 'swingPlannerMultipliers-val :lambda-list '(m))
(cl:defmethod swingPlannerMultipliers-val ((m <mpc_solver_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ocs2_msgs-msg:swingPlannerMultipliers-val is deprecated.  Use ocs2_msgs-msg:swingPlannerMultipliers instead.")
  (swingPlannerMultipliers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mpc_solver_data>) ostream)
  "Serializes a message object of type '<mpc_solver_data>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'initTime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'initState))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'initState))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'finalTime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'modeSchedule) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'targetTrajectories) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mpc_flattened_controller) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'swingPlannerMultipliers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'swingPlannerMultipliers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mpc_solver_data>) istream)
  "Deserializes a message object of type '<mpc_solver_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initTime) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'initState) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'initState)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'finalTime) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'modeSchedule) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'targetTrajectories) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mpc_flattened_controller) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'swingPlannerMultipliers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'swingPlannerMultipliers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mpc_solver_data>)))
  "Returns string type for a message object of type '<mpc_solver_data>"
  "ocs2_msgs/mpc_solver_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mpc_solver_data)))
  "Returns string type for a message object of type 'mpc_solver_data"
  "ocs2_msgs/mpc_solver_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mpc_solver_data>)))
  "Returns md5sum for a message object of type '<mpc_solver_data>"
  "5d8e9cf6f256a05764e6ff2f6f246a7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mpc_solver_data)))
  "Returns md5sum for a message object of type 'mpc_solver_data"
  "5d8e9cf6f256a05764e6ff2f6f246a7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mpc_solver_data>)))
  "Returns full string definition for message of type '<mpc_solver_data>"
  (cl:format cl:nil "float64 initTime           # 对应 scalar_t initTime_~%float64[] initState        # 对应 vector_t initState_~%float64 finalTime          # 对应 scalar_t finalTime_~%~%# ModeSchedule 和 TargetTrajectories ~%mode_schedule modeSchedule~%mpc_target_trajectories targetTrajectories~%~%# 包含 PrimalSolution ~%mpc_flattened_controller mpc_flattened_controller~%~%# swingplanner存储的swingPlannerMultipliers~%float64[] swingPlannerMultipliers~%~%================================================================================~%MSG: ocs2_msgs/mode_schedule~%# MPC mode sequence~%~%float64[] eventTimes           # event times: its size is equal to the size of phaseSequence minus one~%int8[]    modeSequence         # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}~%================================================================================~%MSG: ocs2_msgs/mpc_target_trajectories~%# MPC target trajectories~%~%float64[]    timeTrajectory        # MPC target time trajectory~%mpc_state[]  stateTrajectory       # MPC target state trajectory~%mpc_input[]  inputTrajectory       # MPC target input trajectory~%~%~%================================================================================~%MSG: ocs2_msgs/mpc_state~%# MPC internal model state vector~%float32[] value~%~%================================================================================~%MSG: ocs2_msgs/mpc_input~%# MPC internal model input vector ~%~%float32[] value~%================================================================================~%MSG: ocs2_msgs/mpc_flattened_controller~%# Flattened controller: A serialized controller~%~%# define controllerType Enum values~%uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero~%uint8 CONTROLLER_FEEDFORWARD=1~%uint8 CONTROLLER_LINEAR=2~%~%uint8                   controllerType         # what type of controller is this~%~%mpc_observation         initObservation        # plan initial observation~%~%mpc_target_trajectories planTargetTrajectories # target trajectory in cost function~%mpc_state[]             stateTrajectory        # optimized state trajectory from planner~%mpc_input[]             inputTrajectory        # optimized input trajectory from planner~%float64[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory~%uint16[]                postEventIndices       # array of indices indicating the index of post-event time in the trajectories~%mode_schedule           modeSchedule           # optimal/predefined MPC mode sequence and event times~%~%controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step~%~%mpc_performance_indices performanceIndices     # solver performance indices~%~%================================================================================~%MSG: ocs2_msgs/mpc_observation~%# MPC observation~%float64        time        # Current time~%mpc_state      state       # Current state~%mpc_input      input       # Current input~%int8           mode        # Current mode~%~%================================================================================~%MSG: ocs2_msgs/controller_data~%float32[] data~%~%================================================================================~%MSG: ocs2_msgs/mpc_performance_indices~%# MPC performance indices~%float32     initTime~%float32     merit~%float32     cost~%float32     dynamicsViolationSSE~%float32     equalityConstraintsSSE~%float32     equalityLagrangian~%float32     inequalityLagrangian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mpc_solver_data)))
  "Returns full string definition for message of type 'mpc_solver_data"
  (cl:format cl:nil "float64 initTime           # 对应 scalar_t initTime_~%float64[] initState        # 对应 vector_t initState_~%float64 finalTime          # 对应 scalar_t finalTime_~%~%# ModeSchedule 和 TargetTrajectories ~%mode_schedule modeSchedule~%mpc_target_trajectories targetTrajectories~%~%# 包含 PrimalSolution ~%mpc_flattened_controller mpc_flattened_controller~%~%# swingplanner存储的swingPlannerMultipliers~%float64[] swingPlannerMultipliers~%~%================================================================================~%MSG: ocs2_msgs/mode_schedule~%# MPC mode sequence~%~%float64[] eventTimes           # event times: its size is equal to the size of phaseSequence minus one~%int8[]    modeSequence         # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}~%================================================================================~%MSG: ocs2_msgs/mpc_target_trajectories~%# MPC target trajectories~%~%float64[]    timeTrajectory        # MPC target time trajectory~%mpc_state[]  stateTrajectory       # MPC target state trajectory~%mpc_input[]  inputTrajectory       # MPC target input trajectory~%~%~%================================================================================~%MSG: ocs2_msgs/mpc_state~%# MPC internal model state vector~%float32[] value~%~%================================================================================~%MSG: ocs2_msgs/mpc_input~%# MPC internal model input vector ~%~%float32[] value~%================================================================================~%MSG: ocs2_msgs/mpc_flattened_controller~%# Flattened controller: A serialized controller~%~%# define controllerType Enum values~%uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero~%uint8 CONTROLLER_FEEDFORWARD=1~%uint8 CONTROLLER_LINEAR=2~%~%uint8                   controllerType         # what type of controller is this~%~%mpc_observation         initObservation        # plan initial observation~%~%mpc_target_trajectories planTargetTrajectories # target trajectory in cost function~%mpc_state[]             stateTrajectory        # optimized state trajectory from planner~%mpc_input[]             inputTrajectory        # optimized input trajectory from planner~%float64[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory~%uint16[]                postEventIndices       # array of indices indicating the index of post-event time in the trajectories~%mode_schedule           modeSchedule           # optimal/predefined MPC mode sequence and event times~%~%controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step~%~%mpc_performance_indices performanceIndices     # solver performance indices~%~%================================================================================~%MSG: ocs2_msgs/mpc_observation~%# MPC observation~%float64        time        # Current time~%mpc_state      state       # Current state~%mpc_input      input       # Current input~%int8           mode        # Current mode~%~%================================================================================~%MSG: ocs2_msgs/controller_data~%float32[] data~%~%================================================================================~%MSG: ocs2_msgs/mpc_performance_indices~%# MPC performance indices~%float32     initTime~%float32     merit~%float32     cost~%float32     dynamicsViolationSSE~%float32     equalityConstraintsSSE~%float32     equalityLagrangian~%float32     inequalityLagrangian~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mpc_solver_data>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'initState) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'modeSchedule))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'targetTrajectories))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mpc_flattened_controller))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'swingPlannerMultipliers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mpc_solver_data>))
  "Converts a ROS message object to a list"
  (cl:list 'mpc_solver_data
    (cl:cons ':initTime (initTime msg))
    (cl:cons ':initState (initState msg))
    (cl:cons ':finalTime (finalTime msg))
    (cl:cons ':modeSchedule (modeSchedule msg))
    (cl:cons ':targetTrajectories (targetTrajectories msg))
    (cl:cons ':mpc_flattened_controller (mpc_flattened_controller msg))
    (cl:cons ':swingPlannerMultipliers (swingPlannerMultipliers msg))
))
