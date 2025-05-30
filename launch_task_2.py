import sys
import os
import time
import numpy as np  # 添加 numpy 导入

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tongverse.env import Env, app
from demo.DemoController import DemoController


TASK_SEED = None


def main():
    # 初始化控制器
    controller = DemoController()

    # 启动命令行命令
    controller.start_launch()
    print("start launch")

    # Initialize the environment with task ID and seed.
    env = Env(TASK_SEED, task_id="TaskTwo")
    # Always call env.reset() to initialize the environment.
    env.reset()
    env._kuavo.set_world_pose(
        position=(1, 6, 0.56)
    )
    # task_params = env.get_task_params()
    # print("TASK INFO")
    # print(task_params)

    # agent_params = env.get_robot_params()
    # print("AGENT INFO")
    # print(f"{agent_params}")

    action = {
        "arms": {
            "ctrl_mode": "position",
            "joint_values": [0.0] * 14,
            "stiffness": [50.0, 0, 0, 0, 50.0, 0, 0, 0, 50.0, 0, 0, 0, 0, 0],
            "dampings": [0.0] * 14,
        },
        "legs": {
            "ctrl_mode": "effort",
            "joint_values": np.zeros(12),
            "stiffness": None,
            "dampings": None,
        },
        "head": {
            "ctrl_mode": "position",
            "joint_values": np.zeros(2),
            "stiffness": None,
            "dampings": None,
        },
        "pick": "left_hand",
    }

    def get_obj_pose(env):
        task = env._current_tasks.get("TaskTwo")  
    
        for i in range(1, 4):
            name = f"{task._target_objs}_{i:02}"
            pose = env.scene.get_object(name).get_world_pose() 
            print(pose)
        for dis_name in task._distractor_objs:
            for i in range(1, 4):
                name = f"{dis_name}_{i:02}"
                pose = env.scene.get_object(name).get_world_pose() 
                print(pose)
        return 
    get_obj_pose(env) 
    
    i = 0
    while app.is_running():
        # 执行一步仿真获取观测数据
        obs, is_done = env.step(action)
        
        # 处理观测数据并发布，同时等待新的action
        action = controller.get_action(obs)
        # print("get action")
        # 检查任务是否完成
        if is_done:
            print(obs["extras"])
            break
        i += 1
    controller.cleanup()


if __name__ == "__main__":
    main()
    app.close()
