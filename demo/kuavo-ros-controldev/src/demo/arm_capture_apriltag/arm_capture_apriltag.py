#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import AprilTagDetectionArray
import math
import numpy as np  # 引入numpy库用于数值计算

import time
import argparse
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam

from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.msg import robotHeadMotionData


####################################################################

# 自定义ik参数
use_custom_ik_param = True
# 使用默认的关节角度作为ik的初始预测
joint_angles_as_q0 = False 
# 创建ikSolverParam对象
ik_solve_param = ikSolveParam()
# 设置ikSolveParam对应参数
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 100
ik_solve_param.oritation_constraint_tol= 1e-3
ik_solve_param.pos_constraint_tol = 1e-3 
ik_solve_param.pos_cost_weight = 0.0 

# 手部开合控制
close_hand = [100, 100, 80, 75, 75, 75]    # catch pose
open_hand = [0, 100, 0, 0, 0, 0]          # open pose

# 头部抬头低头控制
def set_head_target(yaw, pitch):
    """
    设置头部目标位置，并发布消息
    :param yaw: 头部的偏航角，范围为[-30, 30]度
    :param pitch: 头部的俯仰角，范围为[-25, 25]度
    """
    
    # 创建一个发布者，发布到'/robot_head_motion_data'话题
    # 使用robotHeadMotionData消息类型，队列大小为10
    pub_head_pose = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建一个robotHeadMotionData消息对象
    head_target_msg = robotHeadMotionData()
    
    # 设置关节数据，包含偏航和俯仰角
    # 确保yaw在[-30, 30]范围内，pitch在[-25, 25]范围内
    head_target_msg.joint_data = [yaw, pitch]
    
    # 发布消息到指定话题
    pub_head_pose.publish(head_target_msg)
    
    # 打印日志信息，显示已发布的头部目标位置
    rospy.loginfo(f"Published head target: yaw={yaw}, pitch={pitch}")

######################## ik求解部分 ############################################

# 获取机器人版本
def get_parameter(param_name):
    try:
        # 获取参数值
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"参数 {param_name} 的值为: {param_value}")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"参数 {param_name} 不存在！程序退出。")
        rospy.signal_shutdown("参数获取失败") 
        return None

# IK 逆解服务
def call_ik_srv(eef_pose_msg):
    # 确保要调用的服务可用
    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    try:
        # 初始化服务代理
        ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
        # 调取服务并获得响应
        res = ik_srv(eef_pose_msg)
        # 返回逆解结果
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False, []

# 设置手臂运动模式
def set_arm_control_mode(mode):
    # 创建服务代理，用于与服务通信
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # 创建请求对象
    request = changeArmCtrlModeRequest()
    request.control_mode = mode  # 设置请求的控制模式

    # 发送请求并接收响应
    response = arm_traj_change_mode_client(request)

    if response.result:
        # 如果响应结果为真，表示成功更改控制模式
        rospy.loginfo(f"Successfully changed arm control mode to {mode}: {response.message}")
    else:
        # 如果响应结果为假，表示更改控制模式失败
        rospy.logwarn(f"Failed to change arm control mode to {mode}: {response.message}")

# 发布手臂目标姿态
def publish_arm_target_poses(times, values):
    # 创建Publisher对象
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    # 创建消息对象并设置传入的times和values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("正在将手臂目标姿态发布到话题 'kuavo_arm_target_poses'")

    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    pub.publish(msg)
    rospy.loginfo("消息已发布。")

# 通过角度（弧度制）计算四元数
class Quaternion:
    def __init__(self):
        self.w = 0    
        self.x = 0    
        self.y = 0     
        self.z = 0

def ToQuaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)

    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
 
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm > 0:
        q.w /= norm
        q.x /= norm
        q.y /= norm
        q.z /= norm

    return q

######################### 识别apriltag标签部分 ###########################################

class AprilTagProcessor:
    def __init__(self, is_init=False):
        """
        初始化AprilTagProcessor类。
        :param is_init: 是否初始化ROS节点,默认为False。
        """
        if is_init:
            rospy.init_node('tag_detections_listener', anonymous=True)

    def quaternion_to_euler(self, w, x, y, z):
        """
        将四元数转换为欧拉角（yaw）。
        :param w, x, y, z: 四元数的分量。
        :return: 以度为单位的yaw角。
        """
        # 计算roll, pitch, yaw
        sinr_cosp = 2 * (w * z + x * y)
        cosr_cosp = 1 - 2 * (y**2 + z**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        #print(f"roll: {math.degrees(roll)}")
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        #print(f"pitch: {math.degrees(pitch)}")
        siny_cosp = 2 * (w * x + y * z)
        cosy_cosp = 1 - 2 * (x**2 + y**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        #print(f"yaw: {math.degrees(yaw)}")
        return math.degrees(yaw)

    def get_apriltag_data(self):
        """
        从指定的ROS话题中获取AprilTag检测数据。
        :return: 包含每个AprilTag信息的列表。
        """
        try:
            # 等待从话题"/robot_tag_info"接收到AprilTagDetectionArray消息
            msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr(f"未能获取到 AprilTag 数据: {e}")
            return None

        data_list = []
        for detection in msg.detections:
            id = detection.id[0]  # 获取AprilTag的ID
            quaternion = detection.pose.pose.pose.orientation  # 获取姿态的四元数
            pos = detection.pose.pose.pose.position  # 获取位置

            # 将四元数转换为yaw角
            yaw_angle = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            
            # 构建AprilTag数据字典
            tag_data = {
                "id": id,
                "off_horizontal": round(pos.x, 3),
                "off_camera": round(pos.y, 3),
                "off_vertical": round(pos.z, 3),
                "yaw_angle": yaw_angle
            }
            data_list.append(tag_data)

        return data_list

    def get_apriltag_by_id(self, tag_id):
        """
        根据ID获取特定的AprilTag数据。
        :param tag_id: 要查找的AprilTag的ID。
        :return: 匹配的AprilTag数据字典。
        """
        all_tags = self.get_apriltag_data()
        if all_tags is None:
            rospy.logerr("未能获取到 AprilTag 数据")
            return None

        for tag in all_tags:
            if tag["id"] == tag_id:
                return tag

        return None

    def get_averaged_apriltag_data(self, tag_id, num_samples=10):
        """
        获取指定ID的AprilTag的平均位置和姿态数据。
        :param tag_id: 要查找的AprilTag的ID。
        :param num_samples: 用于计算平均值的样本数量，默认为10。
        :return: 包含平均位置和姿态的字典。
        """
        data_list = []

        while len(data_list) < num_samples:
            if rospy.is_shutdown():
                return None
            tag_data = self.get_apriltag_by_id(tag_id)
            if tag_data:
                data_list.append(tag_data)
            else :
                rospy.loginfo(f"未检测到AprilTag ID {tag_id}，等待中...")
                time.sleep(0.1) 

        # 使用numpy计算平均值
        avg_off_horizontal = np.mean([tag["off_horizontal"] for tag in data_list])
        avg_off_camera = np.mean([tag["off_camera"] for tag in data_list])
        avg_off_vertical = np.mean([tag["off_vertical"] for tag in data_list])
        avg_yaw_angle = np.mean([tag["yaw_angle"] for tag in data_list])
        
        #print(f"avg_yaw_angle: {avg_yaw_angle}")
        result = {
            "id": tag_id,
            "avg_off_horizontal": round(avg_off_horizontal, 3),
            "avg_off_camera": round(avg_off_camera, 3),
            "avg_off_vertical": round(avg_off_vertical, 3),
            "avg_yaw_angle": round(avg_yaw_angle, 3)
        }
        rospy.loginfo(f"AprilTag ID: {result['id']}, 位置: x={result['avg_off_horizontal']}, y={result['avg_off_camera']}, z={result['avg_off_vertical']}, 倾斜角: {result['avg_yaw_angle']}")
        return result

########################### 主函数 #########################################

def main():

    # 创建AprilTagProcessor实例 并初始化ROS节点
    processor = AprilTagProcessor(is_init=True)

    # 解析命令行参数  
    parser = argparse.ArgumentParser(description="是否启用偏移量")
    parser.add_argument("--offset_start", type=str, choices=["False", "True"], required="True", help="选择 offset_start = True or Flase")
    args = parser.parse_args()

    # offset_start="True"表示启用偏移量 否则不启用偏移量
    if args.offset_start == "True":
        # 偏向侧后边一点
        offset_z=-0.10  # 抓取点位于标签正下方
        temp_x_l=-0.035
        temp_y_l=0.035
        temp_x_r=-0.045
        temp_y_r=0.035
    else :
        offset_z=0.00
        temp_x_l=0.00
        temp_y_l=0.00
        temp_x_r=0.00
        temp_y_r=0.00

    # 角度偏移量（修正绕z轴的偏移角度）
    offset_angle=1.00

    # 低头
    set_head_target(0, 20)
    print("head down")
    time.sleep(2)
    
    # 获取指定ID的AprilTag的平均数据
    tag_data = processor.get_averaged_apriltag_data(tag_id=0)

    # 检测二维码期间 按ctrl+c可以退出程序
    if rospy.is_shutdown():
        return None

    # 判断左手还是右手 后续都会根据这个参数进行判断
    # position_flag > 0 为左手，否则为右手
    if False:
        position_flag=-1
    else :
        position_flag=tag_data['avg_off_camera']*100
        
    print(f"tag position: {position_flag}")

    # 获取机器人版本
    robot_version = get_parameter('robot_version')
    #不同型号机器人的初始位置 (机器人坐标系)
    if robot_version == 45:
        robot_zero_x = -0.0173
        robot_zero_y = -0.2927
        robot_zero_z = -0.2837

    elif robot_version == 42:
        robot_zero_x = -0.0175
        robot_zero_y = -0.25886
        robot_zero_z = -0.20115

    # 设置手臂运动模式为外部控制
    set_arm_control_mode(2)

    # 手部控制api
    # 初始化话题发布者
    hand_control_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    # 创建消息对象
    hand_control_msg = robotHandPosition()

    # 手部松开
    hand_control_msg.left_hand_position =open_hand  # 左手位置   
    hand_control_msg.right_hand_position = open_hand  # 右手位置
    hand_control_pub.publish(hand_control_msg)  # 发布消息

########################################## 运动控制 ik求解 #########################################
    # 创建请求对象
    eef_pose_msg = twoArmHandPoseCmd()
    # 设置请求参数
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # joint_angles_as_q0 为 False 时，这两个参数不会被使用（单位：弧度）
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

    # 抓取位置修正
    if  position_flag > 0 :
        # 设置左手末端执行器的位置
        set_x=tag_data['avg_off_horizontal']+temp_x_l
        set_y=tag_data['avg_off_camera']+temp_y_l
        set_z=tag_data['avg_off_vertical']+offset_z
    else :
        # 设置右手末端执行器的位置
        set_x=tag_data['avg_off_horizontal']+temp_x_r
        set_y=tag_data['avg_off_camera']-temp_y_r
        set_z=tag_data['avg_off_vertical']+offset_z

    # 根据左右手 计算ik参数
    if  position_flag > 0 :
        # 设置左手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([set_x,set_y,set_z])
        #计算末端相对角度
        relative_angle= math.atan((robot_zero_y-set_y)/(set_x-robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        #计算四元数
        quat=ToQuaternion(relative_angle*offset_angle, -1.57 , 0)
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.left_pose.quat_xyzw = [-0.4996018366446333, -0.49999984146591725, 0.49999984146591725, 0.5003981633553666]  # 水平状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        
        # 右手为机器人初始位置
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([ robot_zero_x, robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0,0.0,0.0,1.0]  # 竖直状态
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    else :
        # 左手为机器人初始位置 
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([ robot_zero_x, -1*robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0,0.0,0.0,1.0]   # 竖直状态
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        # 设置右手末端执行器的位置和姿态
        # 使用set_xyz
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([set_x,set_y,set_z])
        # 计算末端相对角度
        relative_angle=math.atan((set_y-robot_zero_y)/(set_x-robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        # 计算四元数
        quat=ToQuaternion(relative_angle*offset_angle, -1.57 , 0.2)
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [quat.x,quat.y,quat.z,quat.w]  # 带yaw角
        #eef_pose_msg.hand_poses.right_pose.quat_xyzw =[0.4996018366446333, -0.49999984146591725, -0.49999984146591725, 0.5003981633553666]  # 水平状态
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    
    print("抓取点x y z")
    print(set_x," , ",set_y," , ",set_z)

    # 调用 IK 逆解服务
    res = call_ik_srv(eef_pose_msg)

    # 逆解成功
    if(res.success):
########################################## 运动控制 准备姿态 #########################################
        
        # 初始位置
        print("move to position 0")
        publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
        20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
        print("回到初始位置")

        time.sleep(0.5)
        
        # 到达等待位置
        if  position_flag > 0 :
            # 打开臂展
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # 弯曲肘部
            print("move to position 2")
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            # 回收臂展 
            #print("move to position 3")
            #publish_arm_target_poses([1.5], [-2.0, 5.0, 0.0, -82.0, -35.0, -7.0, 0.0,
            #20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            #time.sleep(2) 
            print("move over")
        else :
            # 打开臂展
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # 弯曲肘部
            print("move to position 2")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            -10.0, -60.0, 0.0, -90.0, 35.0, 20.0, 0.0])
            #0.0, -60.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            #time.sleep(3) 
            # 回收臂展 
            #print("move to position 3")
            #publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            #8.0, -10.0, 0.0, -90.0, 35.0, 0.0, 0.0])
            #time.sleep(2) 
            print("move over")

########################################## 运动控制 执行ik结果 #########################################

        l_pos = res.hand_poses.left_pose.pos_xyz
        l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos = res.hand_poses.right_pose.pos_xyz
        r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)
        
        # 打印部分逆解结果
        print(f"time_cost: {res.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
        print(f"left_joint_angles: {res.hand_poses.left_pose.joint_angles}")
        print(f"right_joint_angles: {res.hand_poses.right_pose.joint_angles}")
        print(f"res.q_arm: {res.q_arm}")
        
        # ik结束 执行ik结果
        # 0.35 0.52
        if  position_flag > 0 :
            joint_end_angles = np.concatenate([res.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else :
            joint_end_angles = np.concatenate([ [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res.hand_poses.right_pose.joint_angles])
        #joint_end_angles = res.hand_poses.left_pose.joint_angles + res.hand_poses.right_pose.joint_angles
        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        # 调用函数并传入times和values
        publish_arm_target_poses([3], degrees_list)
        print("完成逆解并根据逆解结果到达定位置")
        time.sleep(3.5)

        print("ik结束")

########################################## 运动控制 递水流程 #########################################        
        if  position_flag > 0 :
            # 手部握紧
            hand_control_msg.left_hand_position =close_hand  # 左手位置   
            hand_control_msg.right_hand_position = open_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息
            time.sleep(1)   
            publish_arm_target_poses([1.5], [-60.0, 0.0, 0.0, -30.0, -20.0, 0.0, 0.0,
                20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2.5)
            # 手部松开
            hand_control_msg.left_hand_position =open_hand  # 左手位置   
            hand_control_msg.right_hand_position = open_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息 
            time.sleep(1) 
            #publish_arm_target_poses([1.5], degrees_list)
            #time.sleep(2)
            #time.sleep(1)     
        else :
            hand_control_msg.left_hand_position =open_hand  # 左手位置   
            hand_control_msg.right_hand_position = close_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息
            time.sleep(1)   
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                -60.0, 0.0, 0.0, -30.0, 20.0, 0.0, 0.0])
            time.sleep(2.5)
            # 手部松开
            hand_control_msg.left_hand_position =open_hand  # 左手位置   
            hand_control_msg.right_hand_position = open_hand  # 右手位置
            hand_control_pub.publish(hand_control_msg)  # 发布消息 
            time.sleep(1) 
            #publish_arm_target_poses([1.5], degrees_list)
            #time.sleep(2) 
            #time.sleep(1)
        print("递水完成")

        # 松手后多等一秒
        time.sleep(1) 
    ########################################## 运动控制 后续处理 #########################################
        # 手臂复位 与前面的角度相对应
        if  position_flag > 0 :
            #publish_arm_target_poses([1.5], [0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            #20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            #time.sleep(1.5) 
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
        else :
            #publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            #0.0, 0.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            #time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            0.0, -60.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
            20.0, -60.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5) 
    # ik失败
    else :
        print("ik失败,程序退出")

    # 回到初始位置
    publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 
                                20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])

    # 恢复抬头
    set_head_target(0, 0)
    print("head reset")

    time.sleep(1.5)    

    # 设置手臂控制模式 恢复为 行走时自动摆手
    set_arm_control_mode(1)

if __name__ == '__main__':
    main()
    