
# 挑战杯-人形机器人专项赛

## Introduction  
Leju-Tongverse Simulation Platform

A "AI + Robotics" virtual training arena that provides a safe and controllable simulation environment for embodied intelligence, enabling robots to fully prepare and train before facing real-world challenges.

## Installation and Docker Build

### System Requirements

Ensure the following dependencies are installed before running the platform:

- [Docker](https://docs.docker.com/engine/install/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [Docker Image Download](https://kuavo.lejurobot.com/docker_images/leju_kuavo_tongverse-challenge-cup-2025/kuavo_tv-docker-release-v2.tar.gz)

### Setting Up Docker

1. Load the Docker image:

   ```shell
   docker load -i kuavo_tv-docker-release-v2.tar.gz
   ```

2. Launch the TongVerse-Lite environment:

   ```shell
   # Navigate to the leju_kuavo_tongverse-challenge-cup-2025/ folder
   bash script/run_docker.sh
   ```

## Running a Demo

Inside the Docker environment, start the demo with:

```shell
python launch_task_1.py
```

Note: In `launch_task_1.py`, when constructing the environment:
```bash
"""
Initializes the task environment.

Env Parameters:
    seed (int): Random seed 
    task_id (str): Identifier for the initial task to run (e.g., "TaskOne", "TaskTwo"; default is "TaskOne").
    decimation (int): Number of render and task-checker updates per physics timestep (default is 30).
    start_time (float): Accumulated simulation time in minutes (default is 0).
    Only applicable if task_id is "TaskTwo".
        - If start_time is greater than zero, it will be used as the starting time.
        - If start_time is zero, the system will attempt to load the time from "score.yaml" 
        as a fallback.
"""
!!! Participants are only allowed to modify the decimation parameter to adjust simulation speed.
```
## API Usage

### Task Description:
-----------------
To retrieve task parameters:

```python
task_params = env.get_task_params()
```

Task parameters include:

- **task_goal** (*Dict[str, Dict]*): Task Description
- **camera_intrinsics** (*np.ndarray*): Intrinsic matrix of the camera used in the environment.  
- **camera_resolution** (*Tuple[int]*): Resolution of the camera used in the environment.  


To retrieve robot parameters:

```python
robot_params = env.get_robot_params()
```

Robot parameters include:

```python
{
    "ordered_joint_name": List[str],  # Ordered joint names
    "arm_idx": List[int],  # Arm joint ordered indices
    "leg_idx": List[int],  # Leg joint ordered indices
    "head_idx": List[int],  # Head joint ordered indices
    "start_world_pose": Tuple[np.ndarray, np.ndarray]  # Initial world pose of the robot (position, orientation)
}
```
------

### Action Format

An action is structured as follows:

```python
action = {
    "arms": {
        "ctrl_mode": str,  # Control mode: "position" , "effort", "velocity"
        "joint_values": Optional[Union[np.ndarray, List]],  # Target joint values (shape: 14, ordered)
        "stiffness": Optional[Union[np.ndarray, List]],  # Stiffness values (shape: 14, ordered)
        "dampings": Optional[Union[np.ndarray, List]],  # Damping values (shape: 14, ordered)
    },
    "legs": {
        "ctrl_mode": str,  # Control mode: "position" , "effort", "velocity"
        "joint_values": Optional[Union[np.ndarray, List]],  # Target joint values (shape: 12, ordered)
        "stiffness": Optional[Union[np.ndarray, List]],  # Stiffness values (shape: 12, ordered)
        "dampings": Optional[Union[np.ndarray, List]],  # Damping values (shape: 12, ordered)
    },
    "head": {
        "ctrl_mode": str,  # Control mode: "position" , "effort", "velocity"
        "joint_values": Optional[Union[np.ndarray, List]],  # Target joint values (shape: 2, ordered)
        "stiffness": Optional[Union[np.ndarray, List]],  # Stiffness values (shape: 2, ordered)
        "dampings": Optional[Union[np.ndarray, List]],  # Damping values (shape: 2, ordered)
    },
    "pick": Optional[str], # Optional. One of: "left_hand", "right_hand", or None. Only valid in TaskTwo.  
     
}
```

**Parameters:**

- **ctrl_mode** (*str*): The control mode must be 'position', 'velocity', or 'effort' (case-sensitive).
- **joint_values** (*np.ndarray* or *List* or *None*): Target joint values, must follow joint order.
- **stiffness** (*np.ndarray* or *List* or *None*): Stiffness values, must follow joint order.
- **dampings** (*np.ndarray* or *List* or *None*): Damping values, must follow joint order.

**Notes:**

- The order of `joint_values`, `stiffness`, and `dampings` must match the joint index order.
- You can check the correct joint order using `env.get_robot_params()`.
- The "pick" command is only available during TaskTwo.One of ("left_hand", "right_hand" or None)

    "left_hand": Pick or hold an object with the left hand.

    "right_hand": Pick or hold an object with the right hand.

    None: Do not pick any object, or release the currently held object.

    **Only one object can be picked at a time** 

    Using both hands simultaneously (even for different objects) is not supported.


**Example Usage:**

```python
    action = {
        "arms": {
            "ctrl_mode": "position",
            "joint_values": [0.5, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 14 arm joints
            "stiffness": [50.0, 0, 0, 0, 50.0, 0, 0, 0, 50.0, 0, 0, 0, 0, 0],
            "dampings": [0.0] * 14,
        },
        "legs": {
            "ctrl_mode": "effort",
            "joint_values": np.array([1.0, -1.0, 0.5, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 12 leg joints
            "stiffness": None,  # Not setting stiffness
            "dampings": None,  # Not setting dampings
        },
        "head": {
            "ctrl_mode": "position",
            "joint_values": np.array([1.0, -1.0]),  # 2 head joints
            "stiffness": None,  # Not setting stiffness
            "dampings": None,  # Not setting dampings
        },
        "pick": "left_hand",  # pick object with left hand 
    }
```

-------

### Observation Format:

Observations are structured as follows:

```  python
    obs = { 
        "Kuavo": {
            "body_state": {
                "world_position": np.ndarray,  # (3,) - Robot's world position
                "world_orient": np.ndarray,  # (4,) - World orientation (quaternion)
                "root_linear_velocity": np.ndarray,  # (3,) - Linear velocity of the robot’s root
                "root_angular_velocity": np.ndarray,  # (3,) - Angular velocity of the robot’s root
            },
            "joint_state": {
                "arms": {
                    "positions": np.ndarray,  # (14,) - Arm joint positions
                    "velocities": np.ndarray,  # (14,) - Arm joint velocities
                    "applied_effort": np.ndarray,  # (14,) - Applied effort to arms
                    "stiffness": np.ndarray,  # (14,) - Arm joint stiffness values
                    "dampings": np.ndarray,  # (14,) - Arm joint damping values
                },
                "legs": {
                    "positions": np.ndarray,  # (12,) - Leg joint positions
                    "velocities": np.ndarray,  # (12,) - Leg joint velocities
                    "applied_effort": np.ndarray,  # (12,) - Applied effort to legs
                    "stiffness": np.ndarray,  # (12,) - Leg joint stiffness values
                    "dampings": np.ndarray,  # (12,) - Leg joint damping values
                },
                "head": {
                    "positions": np.ndarray,  # (2,) - Head joint positions
                    "velocities": np.ndarray,  # (2,) - Head joint velocities
                    "applied_effort": np.ndarray,  # (2,) - Applied effort to head
                    "stiffness": np.ndarray,  # (2,) - Head joint stiffness values
                    "dampings": np.ndarray,  # (2,) - Head joint damping values
                }
            }
        },
        "pick": bool,  
        "camera": {
            "rgb": Optional[np.ndarray],  # Optional, (N, 3) - RGB color data per frame
            "depth": Optional[np.ndarray],  # Optional, (n, m, 1) - Depth data per frame
            "world_pose": Optional[Tuple[np.ndarray, np.ndarray]],  # Optional, (position (3,), quaternion (4,))  
        }, 
        "imu_data": {  # The IMU sensor is mounted on the base_link of Kuavo.
            "imu_time": float,
            "linear_acceleration": List[float],   
            "angular_velocity":  List[float],   
            "orientation":  List[float],   
        },
        "extras": { 
            "Current_Task_ID": str, # Current task, one of "TaskOne", "TaskTwo", "TaskThree"
            "time(minutes)": float, # Accumulated time in minutes
            "scores": { # Score breakdown by task.
                "TaskOne": {"score": int, "details": List[str]},
                "TaskTwo": {"score": int, "details": List[str]},
                "TaskThree": {"score": int, "details": List[str]}，
            "info": # Reason for system exit, such as "Fall detected", "Time limit reached", "Task Interrupted" or "Task is done"
        }}
    }
```
**Notes:**

- Camera and extras information in `obs` are updated every decimation steps.
- camera["world_pose"] uses world axes (+X forward, +Z up). ROS uses a different convention: +Z forward, -Y up. If you're using ROS, convert coordinates from Isaac Sim to ROS accordingly.
- "pick": bool,  Only returned in TaskTwo. 
    True: An object is currently held (i.e., the distance between the hand and the object is within the pickup threshold),
    False: No object is currently picked

- After task is completed, the system will automatically export a score.yaml file. This file contains the task scores and relevant performance metrics.

- [Interface Documentation](https://kuavo.lejurobot.com/beta_manual/basic_usage/kuavo-ros-control/docs/4%E5%BC%80%E5%8F%91%E6%8E%A5%E5%8F%A3/%E6%8E%A5%E5%8F%A3%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/)  
-------

 