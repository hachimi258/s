;; Auto-generated. Do not edit!


(when (boundp 'ocs2_msgs::mpc_solver_data)
  (if (not (find-package "OCS2_MSGS"))
    (make-package "OCS2_MSGS"))
  (shadow 'mpc_solver_data (find-package "OCS2_MSGS")))
(unless (find-package "OCS2_MSGS::MPC_SOLVER_DATA")
  (make-package "OCS2_MSGS::MPC_SOLVER_DATA"))

(in-package "ROS")
;;//! \htmlinclude mpc_solver_data.msg.html


(defclass ocs2_msgs::mpc_solver_data
  :super ros::object
  :slots (_initTime _initState _finalTime _modeSchedule _targetTrajectories _mpc_flattened_controller _swingPlannerMultipliers ))

(defmethod ocs2_msgs::mpc_solver_data
  (:init
   (&key
    ((:initTime __initTime) 0.0)
    ((:initState __initState) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:finalTime __finalTime) 0.0)
    ((:modeSchedule __modeSchedule) (instance ocs2_msgs::mode_schedule :init))
    ((:targetTrajectories __targetTrajectories) (instance ocs2_msgs::mpc_target_trajectories :init))
    ((:mpc_flattened_controller __mpc_flattened_controller) (instance ocs2_msgs::mpc_flattened_controller :init))
    ((:swingPlannerMultipliers __swingPlannerMultipliers) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _initTime (float __initTime))
   (setq _initState __initState)
   (setq _finalTime (float __finalTime))
   (setq _modeSchedule __modeSchedule)
   (setq _targetTrajectories __targetTrajectories)
   (setq _mpc_flattened_controller __mpc_flattened_controller)
   (setq _swingPlannerMultipliers __swingPlannerMultipliers)
   self)
  (:initTime
   (&optional __initTime)
   (if __initTime (setq _initTime __initTime)) _initTime)
  (:initState
   (&optional __initState)
   (if __initState (setq _initState __initState)) _initState)
  (:finalTime
   (&optional __finalTime)
   (if __finalTime (setq _finalTime __finalTime)) _finalTime)
  (:modeSchedule
   (&rest __modeSchedule)
   (if (keywordp (car __modeSchedule))
       (send* _modeSchedule __modeSchedule)
     (progn
       (if __modeSchedule (setq _modeSchedule (car __modeSchedule)))
       _modeSchedule)))
  (:targetTrajectories
   (&rest __targetTrajectories)
   (if (keywordp (car __targetTrajectories))
       (send* _targetTrajectories __targetTrajectories)
     (progn
       (if __targetTrajectories (setq _targetTrajectories (car __targetTrajectories)))
       _targetTrajectories)))
  (:mpc_flattened_controller
   (&rest __mpc_flattened_controller)
   (if (keywordp (car __mpc_flattened_controller))
       (send* _mpc_flattened_controller __mpc_flattened_controller)
     (progn
       (if __mpc_flattened_controller (setq _mpc_flattened_controller (car __mpc_flattened_controller)))
       _mpc_flattened_controller)))
  (:swingPlannerMultipliers
   (&optional __swingPlannerMultipliers)
   (if __swingPlannerMultipliers (setq _swingPlannerMultipliers __swingPlannerMultipliers)) _swingPlannerMultipliers)
  (:serialization-length
   ()
   (+
    ;; float64 _initTime
    8
    ;; float64[] _initState
    (* 8    (length _initState)) 4
    ;; float64 _finalTime
    8
    ;; ocs2_msgs/mode_schedule _modeSchedule
    (send _modeSchedule :serialization-length)
    ;; ocs2_msgs/mpc_target_trajectories _targetTrajectories
    (send _targetTrajectories :serialization-length)
    ;; ocs2_msgs/mpc_flattened_controller _mpc_flattened_controller
    (send _mpc_flattened_controller :serialization-length)
    ;; float64[] _swingPlannerMultipliers
    (* 8    (length _swingPlannerMultipliers)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _initTime
       (sys::poke _initTime (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _initState
     (write-long (length _initState) s)
     (dotimes (i (length _initState))
       (sys::poke (elt _initState i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _finalTime
       (sys::poke _finalTime (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; ocs2_msgs/mode_schedule _modeSchedule
       (send _modeSchedule :serialize s)
     ;; ocs2_msgs/mpc_target_trajectories _targetTrajectories
       (send _targetTrajectories :serialize s)
     ;; ocs2_msgs/mpc_flattened_controller _mpc_flattened_controller
       (send _mpc_flattened_controller :serialize s)
     ;; float64[] _swingPlannerMultipliers
     (write-long (length _swingPlannerMultipliers) s)
     (dotimes (i (length _swingPlannerMultipliers))
       (sys::poke (elt _swingPlannerMultipliers i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _initTime
     (setq _initTime (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _initState
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _initState (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _initState i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _finalTime
     (setq _finalTime (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; ocs2_msgs/mode_schedule _modeSchedule
     (send _modeSchedule :deserialize buf ptr-) (incf ptr- (send _modeSchedule :serialization-length))
   ;; ocs2_msgs/mpc_target_trajectories _targetTrajectories
     (send _targetTrajectories :deserialize buf ptr-) (incf ptr- (send _targetTrajectories :serialization-length))
   ;; ocs2_msgs/mpc_flattened_controller _mpc_flattened_controller
     (send _mpc_flattened_controller :deserialize buf ptr-) (incf ptr- (send _mpc_flattened_controller :serialization-length))
   ;; float64[] _swingPlannerMultipliers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _swingPlannerMultipliers (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _swingPlannerMultipliers i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get ocs2_msgs::mpc_solver_data :md5sum-) "5d8e9cf6f256a05764e6ff2f6f246a7f")
(setf (get ocs2_msgs::mpc_solver_data :datatype-) "ocs2_msgs/mpc_solver_data")
(setf (get ocs2_msgs::mpc_solver_data :definition-)
      "float64 initTime           # 对应 scalar_t initTime_
float64[] initState        # 对应 vector_t initState_
float64 finalTime          # 对应 scalar_t finalTime_

# ModeSchedule 和 TargetTrajectories 
mode_schedule modeSchedule
mpc_target_trajectories targetTrajectories

# 包含 PrimalSolution 
mpc_flattened_controller mpc_flattened_controller

# swingplanner存储的swingPlannerMultipliers
float64[] swingPlannerMultipliers

================================================================================
MSG: ocs2_msgs/mode_schedule
# MPC mode sequence

float64[] eventTimes           # event times: its size is equal to the size of phaseSequence minus one
int8[]    modeSequence         # mode sequence: e.g., for a quadrupedal robot, it is in the set {0, 1,..., 15}
================================================================================
MSG: ocs2_msgs/mpc_target_trajectories
# MPC target trajectories

float64[]    timeTrajectory        # MPC target time trajectory
mpc_state[]  stateTrajectory       # MPC target state trajectory
mpc_input[]  inputTrajectory       # MPC target input trajectory


================================================================================
MSG: ocs2_msgs/mpc_state
# MPC internal model state vector
float32[] value

================================================================================
MSG: ocs2_msgs/mpc_input
# MPC internal model input vector 

float32[] value
================================================================================
MSG: ocs2_msgs/mpc_flattened_controller
# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controllerType         # what type of controller is this

mpc_observation         initObservation        # plan initial observation

mpc_target_trajectories planTargetTrajectories # target trajectory in cost function
mpc_state[]             stateTrajectory        # optimized state trajectory from planner
mpc_input[]             inputTrajectory        # optimized input trajectory from planner
float64[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory
uint16[]                postEventIndices       # array of indices indicating the index of post-event time in the trajectories
mode_schedule           modeSchedule           # optimal/predefined MPC mode sequence and event times

controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step

mpc_performance_indices performanceIndices     # solver performance indices

================================================================================
MSG: ocs2_msgs/mpc_observation
# MPC observation
float64        time        # Current time
mpc_state      state       # Current state
mpc_input      input       # Current input
int8           mode        # Current mode

================================================================================
MSG: ocs2_msgs/controller_data
float32[] data

================================================================================
MSG: ocs2_msgs/mpc_performance_indices
# MPC performance indices
float32     initTime
float32     merit
float32     cost
float32     dynamicsViolationSSE
float32     equalityConstraintsSSE
float32     equalityLagrangian
float32     inequalityLagrangian

")



(provide :ocs2_msgs/mpc_solver_data "5d8e9cf6f256a05764e6ff2f6f246a7f")


