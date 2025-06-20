#pragma once

#include <iostream>
#include "kuavo_common/common/robot_state.h"
#include "kuavo_common/common/sensor_data.h"
#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/json_config_reader.hpp"
#include "kuavo_common/kuavo_common.h"
#include "actuators_interface.h"
#include "kalman_estimate.h"
#include "imu_receiver.h"
#include "utils.h"
#include "ruierman_actuator.h"
#include "ruiwo_actuator.h"
#include "jodell_claw_driver.h"
#include "dynamixel_interface.h"
#include "ankle_solver.h"
#include "hand_controller.h"
#include "lejuclaw_controller.h"
#include "claw_types.h"
#include "gesture_types.h"
#include "touch_hand_controller.h"
#include "hipnuc_imu_receiver.h"

namespace HighlyDynamic
{
#define MOTOR_OFFSET_L (-15 * M_PI / 180)
#define MOTOR_OFFSET_S (-15 * M_PI / 180)
#define CST 0
#define CSV 1
#define CSP 2

inline std::vector<double> eigenToStdVector(const Eigen::VectorXd& vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

struct HardwareParam {
    bool cali_leg{false};
    std::vector<double> default_joint_pos;
    int robot_version{42};
    bool cali{false}; 
    bool cali_arm{false};
    int build_cppad_state{0};
    bool only_half_up_body{false};
    int teach_pendant_{0};
};

enum ImuType
{
IMU_TYPE_NONE = 0,
IMU_TYPE_HIPNUC = 1,
IMU_TYPE_XSENS = 2,
};

class HardwarePlant
{
    struct RuiWoJointData
    {
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> torque;
    };

public:
    HardwarePlant(double dt = 1e-3, HardwareParam hardware_param = HardwareParam(), const std::string & hardware_abs_path = "", uint8_t control_mode = MOTOR_CONTROL_MODE_TORQUE,
                 uint16_t num_actuated = 0,
                 uint16_t nq_f = 7, uint16_t nv_f = 6);
    ~HardwarePlant(){HWPlantDeInit();}
    void Update(RobotState_t state_des, Eigen::VectorXd actuation);
    void joint2motor(const RobotState_t &state_des_, const Eigen::VectorXd &actuation, Eigen::VectorXd &cmd_out);
    void motor2joint(SensorData_t sensor_data_motor, SensorData_t &sensor_data_joint);
    void GetC2Tcoeff(double *ret_c2t_coeff);
    void cmds2Cmdr(const Eigen::VectorXd &cmd_s, uint32_t na_src, Eigen::VectorXd &cmd_r, uint32_t na_r);
    bool readSensor(SensorData_t &sensor_data);
    void setState(SensorData_t &sensor_data_motor, SensorData_t &sensor_data_joint);
    void getState(SensorData_t &sensor_data_motor, SensorData_t &sensor_data_joint);
    bool HWPlantCheck();
    size_t get_num_actuated() const;

    HardwareSettings get_motor_info() const { return motor_info; };
    int8_t HWPlantInit();
    SensorData_t sensorsInitHW();
    bool sensorsCheck();
    void HWPlantDeInit();
    void jointMoveTo(std::vector<double> goal_pos, double speed, double dt = 1e-3, double current_limit=-1);

    void qv_joint_to_motor(Eigen::VectorXd &no_arm_state, Eigen::VectorXd &with_arm_state, uint32_t nq_with_arm, uint32_t nq_no_arm);
    int8_t PDInitialize(Eigen::VectorXd &q0);
    void writeCommand(Eigen::VectorXd cmd_r, uint32_t na_r, std::vector<int> control_modes, Eigen::VectorXd &joint_kp, Eigen::VectorXd &joint_kd);
    void endEffectorCommand(std::vector<EndEffectorData> &end_effector_cmd);
    bool checkJointPos(const std::vector<JointParam_t> &joint_data, std::vector<uint8_t> ids, std::string &msg);
    void jointFiltering(std::vector<JointParam_t> &joint_data, double dt);
    void setDefaultJointPos(std::vector<double> default_joint_pos);
    
    void setEcmasterDriverType(std::string type = "elmo");
    static void signalHandler(int sig);
    bool setCurrentPositionAsOffset();
    void performJointSymmetryCheck();
    inline void SetJointVelocity(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void SetJointTorque(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void SetJointPosition(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    inline void GetJointData(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    bool calibrateMotor(int motor_id, int direction, bool save_offset = false);
    void calibrateLoop();
    void calibrateArmJoints();
    void initEndEffector();

    bool checkLejuClawInitialized();
    bool checkHandInitialized();
    bool executeGestures(const std::vector<eef_controller::GestureExecuteInfo>& gesture_tasks, std::string& err_msg);
    std::vector<eef_controller::GestureInfoMsg> listGestures();
    bool isGestureExecuting();
    bool controlLejuClaw(eef_controller::ControlClawRequest& req, eef_controller::ControlClawResponse& res);
    bool controlLejuClaw(eef_controller::lejuClawCommand& command);
    eef_controller::ClawState getLejuClawState();

    void setHardwareParam(const HardwareParam& param) { hardware_param_ = param; }
    HardwareParam& getHardwareParam() { return hardware_param_; }
    const std::array<eef_controller::BrainCoController::HandStatus, 2>&  getHandControllerStatus();
    eef_controller::FingerStatusPtrArray getHandControllerWithTouchStatus();

    bool th_running_ = false;
    int hardware_status_ = -1;
    bool hardware_ready_ = false;
    bool redundant_imu_ = false; // 冗余imu
    uint32_t num_joint = 0;
    uint32_t num_arm_joints = 0;
    uint32_t num_head_joints = 2;
    char initial_input_cmd_ = '\0';
    std::string end_effector_type_;
    std::vector<double_t> joint_velocity_limits_;
    std::vector<double_t> joint_current_limits_;

    std::unique_ptr<eef_controller::TouchDexhandContrller> dexhand_actuator;
    std::string gesture_filepath_;

private:

    SensorData_t sensor_data_motor_last;
    SensorData_t sensor_data_joint_last;
    std::mutex motor_joint_data_mtx_;
    SensorData_t sensor_data_joint;

    double dt_ = 1e-3;
    uint8_t control_mode_ = MOTOR_CONTROL_MODE_TORQUE;
    uint16_t num_actuated_ = 0;
    uint16_t nq_f_ = 7;
    uint16_t nv_f_ = 6;
    int32_t na_foot_;
    int32_t nq_;
    int32_t nv_;
    std::vector<std::string> end_frames_name_;
    std::unique_ptr<KalmanEstimate> filter;
    bool Uncalibration_IMU = true;

    SensorData_t sensor_data_;
    Eigen::Vector3d free_acc_;
    Decoupled_states decoup_states;
    bool ori_init_flag;
    bool is_cali_{false};
    double ruiwo_motor_velocity_factor_{0.005};
    std::vector<std::string> ruiwo_2_joint_name_;
    std::map<std::string, std::vector<double>> ruiwo_velocity_limit_map_;

    RobotState_t state_est_, prev_state_est_;
    RobotState_t state_des_, prev_state_des_;
    Eigen::Vector3d p_landing_;

    RobotState_t raw_state_est_, raw_prev_state_est_;
    Eigen::Vector3d raw_p_landing_;
    Eigen::Vector4d ankle_motor_offset;
    Eigen::Vector4d arm_ankle_motor_offset_;
    Eigen::VectorXd qv_;
    std::vector<JointParam_t> joint_data;
    std::vector<JointParam_t> joint_data_old;
    std::vector<JointParam_t> joint_cmd;
    std::vector<JointParam_t> joint_cmd_old;
    std::vector<uint8_t> joint_ids;
    std::unordered_map<int, int> ec_index_map_;

    std::vector<double> c2t_coeff;
    std::vector<double_t> min_joint_position_limits;
    std::vector<double_t> max_joint_position_limits;
    KuavoCommon *kuavo_common_ptr_;
    KuavoSettings kuavo_settings_;
    HardwareSettings motor_info;
    bool has_end_effectors{false};
    double q_hip_pitch_prev_;
    RobotVersion robot_version_;
    std::string ecmaster_type_ = "elmo";
    HardwareParam hardware_param_;

    /* only used in half-up body mode */
    std::unique_ptr<std::array<double, 12>> stance_leg_joint_pos_ = nullptr;
};

void Invt_imudate(SensorData_t &sensor_data);
Eigen::Vector3d removeGravity(const Eigen::Vector3d &rawAccel, const Eigen::Quaterniond &orientation);

} // namespace HighlyDynamic