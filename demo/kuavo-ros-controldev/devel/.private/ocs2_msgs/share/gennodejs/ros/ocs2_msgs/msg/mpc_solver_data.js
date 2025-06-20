// Auto-generated. Do not edit!

// (in-package ocs2_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let mode_schedule = require('./mode_schedule.js');
let mpc_target_trajectories = require('./mpc_target_trajectories.js');
let mpc_flattened_controller = require('./mpc_flattened_controller.js');

//-----------------------------------------------------------

class mpc_solver_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.initTime = null;
      this.initState = null;
      this.finalTime = null;
      this.modeSchedule = null;
      this.targetTrajectories = null;
      this.mpc_flattened_controller = null;
      this.swingPlannerMultipliers = null;
    }
    else {
      if (initObj.hasOwnProperty('initTime')) {
        this.initTime = initObj.initTime
      }
      else {
        this.initTime = 0.0;
      }
      if (initObj.hasOwnProperty('initState')) {
        this.initState = initObj.initState
      }
      else {
        this.initState = [];
      }
      if (initObj.hasOwnProperty('finalTime')) {
        this.finalTime = initObj.finalTime
      }
      else {
        this.finalTime = 0.0;
      }
      if (initObj.hasOwnProperty('modeSchedule')) {
        this.modeSchedule = initObj.modeSchedule
      }
      else {
        this.modeSchedule = new mode_schedule();
      }
      if (initObj.hasOwnProperty('targetTrajectories')) {
        this.targetTrajectories = initObj.targetTrajectories
      }
      else {
        this.targetTrajectories = new mpc_target_trajectories();
      }
      if (initObj.hasOwnProperty('mpc_flattened_controller')) {
        this.mpc_flattened_controller = initObj.mpc_flattened_controller
      }
      else {
        this.mpc_flattened_controller = new mpc_flattened_controller();
      }
      if (initObj.hasOwnProperty('swingPlannerMultipliers')) {
        this.swingPlannerMultipliers = initObj.swingPlannerMultipliers
      }
      else {
        this.swingPlannerMultipliers = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mpc_solver_data
    // Serialize message field [initTime]
    bufferOffset = _serializer.float64(obj.initTime, buffer, bufferOffset);
    // Serialize message field [initState]
    bufferOffset = _arraySerializer.float64(obj.initState, buffer, bufferOffset, null);
    // Serialize message field [finalTime]
    bufferOffset = _serializer.float64(obj.finalTime, buffer, bufferOffset);
    // Serialize message field [modeSchedule]
    bufferOffset = mode_schedule.serialize(obj.modeSchedule, buffer, bufferOffset);
    // Serialize message field [targetTrajectories]
    bufferOffset = mpc_target_trajectories.serialize(obj.targetTrajectories, buffer, bufferOffset);
    // Serialize message field [mpc_flattened_controller]
    bufferOffset = mpc_flattened_controller.serialize(obj.mpc_flattened_controller, buffer, bufferOffset);
    // Serialize message field [swingPlannerMultipliers]
    bufferOffset = _arraySerializer.float64(obj.swingPlannerMultipliers, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mpc_solver_data
    let len;
    let data = new mpc_solver_data(null);
    // Deserialize message field [initTime]
    data.initTime = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [initState]
    data.initState = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [finalTime]
    data.finalTime = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [modeSchedule]
    data.modeSchedule = mode_schedule.deserialize(buffer, bufferOffset);
    // Deserialize message field [targetTrajectories]
    data.targetTrajectories = mpc_target_trajectories.deserialize(buffer, bufferOffset);
    // Deserialize message field [mpc_flattened_controller]
    data.mpc_flattened_controller = mpc_flattened_controller.deserialize(buffer, bufferOffset);
    // Deserialize message field [swingPlannerMultipliers]
    data.swingPlannerMultipliers = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.initState.length;
    length += mode_schedule.getMessageSize(object.modeSchedule);
    length += mpc_target_trajectories.getMessageSize(object.targetTrajectories);
    length += mpc_flattened_controller.getMessageSize(object.mpc_flattened_controller);
    length += 8 * object.swingPlannerMultipliers.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ocs2_msgs/mpc_solver_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d8e9cf6f256a05764e6ff2f6f246a7f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 initTime           # 对应 scalar_t initTime_
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mpc_solver_data(null);
    if (msg.initTime !== undefined) {
      resolved.initTime = msg.initTime;
    }
    else {
      resolved.initTime = 0.0
    }

    if (msg.initState !== undefined) {
      resolved.initState = msg.initState;
    }
    else {
      resolved.initState = []
    }

    if (msg.finalTime !== undefined) {
      resolved.finalTime = msg.finalTime;
    }
    else {
      resolved.finalTime = 0.0
    }

    if (msg.modeSchedule !== undefined) {
      resolved.modeSchedule = mode_schedule.Resolve(msg.modeSchedule)
    }
    else {
      resolved.modeSchedule = new mode_schedule()
    }

    if (msg.targetTrajectories !== undefined) {
      resolved.targetTrajectories = mpc_target_trajectories.Resolve(msg.targetTrajectories)
    }
    else {
      resolved.targetTrajectories = new mpc_target_trajectories()
    }

    if (msg.mpc_flattened_controller !== undefined) {
      resolved.mpc_flattened_controller = mpc_flattened_controller.Resolve(msg.mpc_flattened_controller)
    }
    else {
      resolved.mpc_flattened_controller = new mpc_flattened_controller()
    }

    if (msg.swingPlannerMultipliers !== undefined) {
      resolved.swingPlannerMultipliers = msg.swingPlannerMultipliers;
    }
    else {
      resolved.swingPlannerMultipliers = []
    }

    return resolved;
    }
};

module.exports = mpc_solver_data;
