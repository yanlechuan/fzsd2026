

#ifndef MY_DOG_FSM_HPP
#define MY_DOG_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

namespace my_dog_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive"), last_fix_motiontime_(0), motor_enable_motiontime_(-9999), block_a_for_getup_(false), a_released_after_enable_(false), all_motors_confirmed_(false) {}

    void Enter() override
    {
        last_fix_motiontime_ = 0;
        motor_enable_motiontime_ = -9999;
        block_a_for_getup_ = false;
        a_released_after_enable_ = false;
        all_motors_confirmed_ = false;
        motor_last_q_.clear();
        motor_q_changed_.clear();
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0'/A→GetUp, '5'→CalfSwing, '6'/RT_DPadUp→ClimbWall, '7'/RT_DPadDown→Bridge, '8'→Squat." << std::endl;
        std::cout << LOGGER::NOTE << "Press 'LB + B' on gamepad to run robstride_off.sh (motor disable), 'LT + B' to run poweroff.sh (shutdown)." << std::endl;
        std::cout << LOGGER::NOTE << "Press 'LB + A' on gamepad to run fix.sh (motor bringup, min 3s interval)." << std::endl;
        std::cout << LOGGER::NOTE << "After motor enable (LB+A), all 12 motors must be confirmed online before GetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }

        // 电机使能后，通过位置变化检测所有 12 个电机是否在线
        if (block_a_for_getup_ && !all_motors_confirmed_)
        {
            const auto& q = fsm_state->motor_state.q;
            int n = rl.params.Get<int>("num_of_dofs");
            if (static_cast<int>(q.size()) >= n)
            {
                if (motor_last_q_.empty())
                {
                    motor_last_q_ = q;
                    motor_q_changed_.resize(n, false);
                }
                else
                {
                    for (int i = 0; i < n; ++i)
                    {
                        if (!motor_q_changed_[i] && std::abs(q[i] - motor_last_q_[i]) > 0.0001f)
                        {
                            motor_q_changed_[i] = true;
                        }
                    }
                    // 检查是否全部 12 个电机都已检测到位置变化
                    // 互补检测：若位置未变但力矩有波动（使能但编码器恰好未跳变），也算在线
                    // const auto& tau = fsm_state->motor_state.tau_est;
                    // for (int i = 0; i < n; ++i)
                    // {
                    //     if (!motor_q_changed_[i] && static_cast<int>(tau.size()) > i && std::abs(tau[i]) > 0.001f)
                    //     {
                    //         motor_q_changed_[i] = true;
                    //     }
                    // }
                    bool all_changed = true;
                    int online_count = 0;
                    for (int i = 0; i < n; ++i)
                    {
                        if (motor_q_changed_[i]) online_count++;
                        else all_changed = false;
                    }
                    if (all_changed && online_count >= n)
                    {
                        all_motors_confirmed_ = true;
                        std::cout << LOGGER::NOTE << "[Safety] All " << n << " motors confirmed online (position change detected)." << std::endl;
                    }
                    motor_last_q_ = q;
                }
            }
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // LT + B combination: execute poweroff.sh for system shutdown
        if (rl.control.current_gamepad == Input::Gamepad::LT_B)
        {
            std::cout << LOGGER::INFO << "[Gamepad] LT+B pressed - executing poweroff.sh..." << std::endl;
            int ret = system("bash $HOME/rl_sar/src/rl_sar/poweroff.sh &");
            if (ret != 0)
            {
                std::cout << LOGGER::ERROR << "[Gamepad] poweroff.sh execution failed with code: " << ret << std::endl;
            }
        }
        // LB + B combination: execute robstride_off.sh for motor disable
        if (rl.control.current_gamepad == Input::Gamepad::LB_B)
        {
            std::cout << LOGGER::INFO << "[Gamepad] LB+B pressed - executing robstride_off.sh..." << std::endl;
            int ret = system("bash $HOME/rl_sar/src/rl_sar/robstride_off.sh &");
            if (ret != 0)
            {
                std::cout << LOGGER::ERROR << "[Gamepad] robstride_off.sh execution failed with code: " << ret << std::endl;
            }
        }
        // LB + A combination: execute fix.sh for motor bringup (min 3s interval, dt=0.002 -> ~1500 frames)
        if (rl.control.current_gamepad == Input::Gamepad::LB_A &&
            rl.motiontime - last_fix_motiontime_ > 1500)
        {
            last_fix_motiontime_ = rl.motiontime;
            motor_enable_motiontime_ = rl.motiontime;
            block_a_for_getup_ = true;
            a_released_after_enable_ = false;
            all_motors_confirmed_ = false;
            motor_last_q_.clear();
            motor_q_changed_.clear();
            std::cout << LOGGER::INFO << "[Gamepad] LB+A pressed - executing fix.sh..." << std::endl;
            std::cout << LOGGER::WARNING << "[Safety] Motor enable triggered. Waiting for all 12 motors to come online..." << std::endl;
            int ret = system("bash $HOME/rl_sar/src/rl_sar/fix.sh &");
            if (ret != 0)
            {
                std::cout << LOGGER::ERROR << "[Gamepad] fix.sh execution failed with code: " << ret << std::endl;
            }
        }
        // 检测 A 键是否已释放（用于解除 LB+A 组合键的安全锁定）
        // 使用持久化标记 a_released_after_enable_，避免 ClearInput 导致 current_gamepad 不可靠
        if (block_a_for_getup_ && !a_released_after_enable_ &&
            rl.control.current_gamepad != Input::Gamepad::A &&
            rl.control.current_gamepad != Input::Gamepad::LB_A)
        {
            a_released_after_enable_ = true;
            std::cout << LOGGER::NOTE << "[Safety] A button released after motor enable. Press A to GetUp." << std::endl;
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            if (BlockTransitionTo("GetUp")) return state_name_;
            return "RLFSMStateGetUp";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            if (BlockTransitionTo("CalfSwing")) return state_name_;
            return "RLFSMStateCalfSwing";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            if (BlockTransitionTo("ClimbWall")) return state_name_;
            return "RLFSMStateClimbWall";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            if (BlockTransitionTo("Bridge")) return state_name_;
            return "RLFSMStateBridge";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num8)
        {
            if (BlockTransitionTo("Squat")) return state_name_;
            return "RLFSMStateSquat";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStateGetUp"))     { if (!BlockTransitionTo("GetUp"))     return "RLFSMStateGetUp"; }
        if (TryNavStateChange("RLFSMStateSquat"))     { if (!BlockTransitionTo("Squat"))     return "RLFSMStateSquat"; }
        if (TryNavStateChange("RLFSMStateClimbWall")) { if (!BlockTransitionTo("ClimbWall")) return "RLFSMStateClimbWall"; }
        if (TryNavStateChange("RLFSMStateBridge"))    { if (!BlockTransitionTo("Bridge"))    return "RLFSMStateBridge"; }
        return state_name_;
    }

private:
    // 电机使能后，检查是否允许状态转换（需全部电机在线 + A 键释放重按）
    bool BlockTransitionTo(const std::string& target)
    {
        if (!block_a_for_getup_) return false;  // 未触发电机使能，放行

        if (rl.motiontime - motor_enable_motiontime_ < 1500)
        {
            std::cout << LOGGER::WARNING << "[Safety] Motor enable cooldown active ("
                      << (1500 - (rl.motiontime - motor_enable_motiontime_)) * 0.002f
                      << "s remaining). Cannot switch to " << target << " yet." << std::endl;
            return true;
        }
        if (!all_motors_confirmed_)
        {
            int online = 0;
            int total = static_cast<int>(motor_q_changed_.size());
            for (bool changed : motor_q_changed_) if (changed) online++;
            std::cout << LOGGER::WARNING << "[Safety] Motors not all online yet ("
                      << online << "/" << total
                      << " confirmed). Cannot switch to " << target << "." << std::endl;
            return true;
        }
        // 全部电机已确认在线，但还需 A 键曾被释放过（防止 LB+A 松 LB 留 A 的误触发）
        if (!a_released_after_enable_)
        {
            std::cout << LOGGER::WARNING << "[Safety] A button not released since motor enable. "
                      << "Please release A and press again to switch to " << target << "." << std::endl;
            return true;
        }
        return false;
    }

    int last_fix_motiontime_;
    int motor_enable_motiontime_;       // 记录电机使能触发时刻
    bool block_a_for_getup_;            // 电机使能后阻止状态转换，直到条件满足
    bool a_released_after_enable_;      // 电机使能后 A 键是否已被释放过（持久化标记）
    bool all_motors_confirmed_;         // 全部 12 电机已确认在线（位置有变化）
    std::vector<float> motor_last_q_;   // 上一帧的电机位置，用于检测位置变化
    std::vector<bool> motor_q_changed_; // 每个电机是否已检测到位置变化
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, 1.00, 0.80,
        -0.00, -1.00, -0.80,
        -0.00, 1.00, 0.90,
        0.00, -1.00, -0.90
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }
        
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num8)
            {
                return "RLFSMStateSquat";
            }
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
            {
                return "RLFSMStateRLLocomotion2";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
            {
                return "RLFSMStateRLLocomotion3";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
            {
                return "RLFSMStateRLLocomotion4";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateSquat"))          return "RLFSMStateSquat";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateRLLocomotion"))   return "RLFSMStateRLLocomotion";
        if (TryNavStateChange("RLFSMStateRLLocomotion2"))  return "RLFSMStateRLLocomotion2";
        if (TryNavStateChange("RLFSMStateRLLocomotion3"))  return "RLFSMStateRLLocomotion3";
        if (TryNavStateChange("RLFSMStateRLLocomotion4"))  return "RLFSMStateRLLocomotion4";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        return state_name_;
    }

private:
    void CheckSafety()
    {
        // check motor offline (manual power cycle, etc.)
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        // check robot attitude
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // check robot torque
        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        // 注意：ROS2标准的四元数顺序是 [x, y, z, w]，而非 [w, x, y, z],但是我们强化学习使用的是w,x,y,z
        float qw = imu.quaternion[0]; // w
        float qx = imu.quaternion[1]; // x
        float qy = imu.quaternion[2]; // y
        float qz = imu.quaternion[3]; // z
        
        // 检查四元数是否标准化
        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            // 标准化四元数
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        
        // 计算欧拉角 (航空zyx顺序，即yaw-pitch-roll)
        float roll, pitch, yaw;
        
        // 计算旋转矩阵元素
        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);
        
        // 从旋转矩阵计算欧拉角
        pitch = -asin(m20);
        
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            // 万向锁情况
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }
        
        // 转换为度
        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;
        
        // 检查姿态阈值
        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;
        
        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2) 
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }
        
        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive")) return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetUp"))   return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))   return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))      return "RLFSMStateBridge";
        return state_name_;
    }
};

class RLFSMStateCalfSwing : public RLFSMState
{
public:
    RLFSMStateCalfSwing(RL *rl) : RLFSMState(*rl, "RLFSMStateCalfSwing") {}

    float percent_transition = 0.0f;
    float swing_time = 0.0f;
    float swing_period_seconds = 8.0f;
    std::vector<float> target_dof_pos;
    std::vector<float> calf_min_pos = {
        -0.0052f, 0.0175f, -0.0136f, 0.0117f
    };
    std::vector<float> calf_max_pos = {
        -1.4042f, 1.4368f, -1.3881f, 1.4629f
    };

    void Enter() override
    {
        percent_transition = 0.0f;
        swing_time = 0.0f;
        target_dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");

        std::vector<float> custom_hip_angles;
        std::vector<float> custom_thigh_angles;
        std::vector<float> custom_calf_min_pos;
        std::vector<float> custom_calf_max_pos;

        if (TryGetVectorParam("custom_hip_angles", custom_hip_angles) && custom_hip_angles.size() >= 4)
        {
            ApplyJointTargets(custom_hip_angles, {3, 0, 9, 6});
        }

        if (TryGetVectorParam("custom_thigh_angles", custom_thigh_angles) && custom_thigh_angles.size() >= 4)
        {
            ApplyJointTargets(custom_thigh_angles, {4, 1, 10, 7});
        }

        if (TryGetVectorParam("calf_swing_min_angles", custom_calf_min_pos) && custom_calf_min_pos.size() >= 4)
        {
            calf_min_pos = custom_calf_min_pos;
        }

        if (TryGetVectorParam("calf_swing_max_angles", custom_calf_max_pos) && custom_calf_max_pos.size() >= 4)
        {
            calf_max_pos = custom_calf_max_pos;
        }

        swing_period_seconds = rl.params.Get<float>("calf_swing_period_seconds", 8.0f);
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        rl.rl_init_done = true;
    }

    void Run() override
    {
        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }

        if (Interpolate(percent_transition, rl.now_state.motor_state.q, target_dof_pos, 3.0f, "Custom pose transition", true)) return;

        const float dt = rl.params.Get<float>("dt");
        swing_time += dt;
        ApplySwingPose();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        // 导航模式下的状态转换（排除 CalfSwing，可从 CalfSwing 切出但不能切回）
        // 注意：CalfSwing 不可直接切步态，必须经过 GetUp 确保站立姿势到位
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetUp"))          return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        return state_name_;
    }

private:
    bool TryGetVectorParam(const std::string& key, std::vector<float>& value)
    {
        try
        {
            value = rl.params.Get<std::vector<float>>(key);
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    void ApplyJointTargets(const std::vector<float>& values, std::initializer_list<int> indices)
    {
        size_t value_index = 0;
        for (int index : indices)
        {
            if (index >= 0 && static_cast<size_t>(index) < target_dof_pos.size() && value_index < values.size())
            {
                target_dof_pos[index] = values[value_index];
            }
            ++value_index;
        }
    }

    void ApplySwingPose()
    {
        auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
        const int num_of_dofs = rl.params.Get<int>("num_of_dofs");
        const int command_dofs = std::min(num_of_dofs, static_cast<int>(target_dof_pos.size()));

        const float period = std::max(swing_period_seconds, 0.1f);
        const float phase = 2.0f * static_cast<float>(M_PI) * swing_time / period;
        const float swing_ratio = 0.5f * (1.0f - std::cos(phase));

        for (int i = 0; i < command_dofs; ++i)
        {
            fsm_command->motor_command.q[i] = target_dof_pos[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }

        const int calf_indices[4] = {5, 2, 11, 8};
        for (int leg = 0; leg < 4; ++leg)
        {
            if (calf_indices[leg] < command_dofs && leg < static_cast<int>(calf_min_pos.size()) && leg < static_cast<int>(calf_max_pos.size()))
            {
                fsm_command->motor_command.q[calf_indices[leg]] = calf_min_pos[leg] + swing_ratio * (calf_max_pos[leg] - calf_min_pos[leg]);
            }
        }
    }

    void CheckSafety()
    {
        // 注意：CalfSwing 不做电机停转检测，因正弦运动在极值处速度为零，会误触发
        // 且该状态仅能通过键盘进入（手柄无法触发），操作者可知晓风险
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        float qw = imu.quaternion[0];
        float qx = imu.quaternion[1];
        float qy = imu.quaternion[2];
        float qz = imu.quaternion[3];

        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }

        float roll, pitch, yaw;

        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);

        pitch = -asin(m20);

        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }

        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;

        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;

        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2)
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }

        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.history_obs_buf.reset({0}, std::vector<float>());
        
        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();

        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLLocomotion2";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLLocomotion3";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLLocomotion4";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        if (TryNavStateChange("RLFSMStateGetUp"))          return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateSquat"))          return "RLFSMStateSquat";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateRLLocomotion"))   return "RLFSMStateRLLocomotion";
        if (TryNavStateChange("RLFSMStateRLLocomotion2"))  return "RLFSMStateRLLocomotion2";
        if (TryNavStateChange("RLFSMStateRLLocomotion3"))  return "RLFSMStateRLLocomotion3";
        if (TryNavStateChange("RLFSMStateRLLocomotion4"))  return "RLFSMStateRLLocomotion4";
        return state_name_;
    }

private:
    void CheckSafety()  // RLFSMStateRLLocomotion
    {
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        // check robot attitude
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // check robot torque
        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        //  [w, x, y, z]
        float qw = imu.quaternion[0]; // x
        float qx = imu.quaternion[1]; // y
        float qy = imu.quaternion[2]; // z
        float qz = imu.quaternion[3]; // w
        
        // 检查四元数是否标准化
        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            // 标准化四元数
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        
        // 计算欧拉角 (航空zyx顺序，即yaw-pitch-roll)
        float roll, pitch, yaw;
        
        // 计算旋转矩阵元素
        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);
        
        // 从旋转矩阵计算欧拉角
        pitch = -asin(m20);
        
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            // 万向锁情况
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }
        
        // 转换为度
        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;
        
        // 检查姿态阈值
        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;
        
        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2) 
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }
        
        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }


};

class RLFSMStateRLLocomotion2 : public RLFSMState
{
public:
    RLFSMStateRLLocomotion2(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion2") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.history_obs_buf.reset({0}, std::vector<float>());

        // read params from yaml
        rl.config_name = "robot_lab2";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();

        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLLocomotion2";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLLocomotion3";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLLocomotion4";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        if (TryNavStateChange("RLFSMStateGetUp"))          return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateRLLocomotion"))   return "RLFSMStateRLLocomotion";
        if (TryNavStateChange("RLFSMStateRLLocomotion2"))  return "RLFSMStateRLLocomotion2";
        if (TryNavStateChange("RLFSMStateRLLocomotion3"))  return "RLFSMStateRLLocomotion3";
        if (TryNavStateChange("RLFSMStateRLLocomotion4"))  return "RLFSMStateRLLocomotion4";
        return state_name_;
    }

private:
    void CheckSafety()  // RLFSMStateRLLocomotion2
    {
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        // check robot attitude
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // check robot torque
        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        //  [w, x, y, z]
        float qw = imu.quaternion[0]; // x
        float qx = imu.quaternion[1]; // y
        float qy = imu.quaternion[2]; // z
        float qz = imu.quaternion[3]; // w
        
        // 检查四元数是否标准化
        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            // 标准化四元数
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        
        // 计算欧拉角 (航空zyx顺序，即yaw-pitch-roll)
        float roll, pitch, yaw;
        
        // 计算旋转矩阵元素
        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);
        
        // 从旋转矩阵计算欧拉角
        pitch = -asin(m20);
        
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            // 万向锁情况
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }
        
        // 转换为度
        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;
        
        // 检查姿态阈值
        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;
        
        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2) 
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }
        
        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }


};

class RLFSMStateRLLocomotion3 : public RLFSMState
{
public:
    RLFSMStateRLLocomotion3(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion3") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.history_obs_buf.reset({0}, std::vector<float>());

        // read params from yaml
        rl.config_name = "robot_lab3";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();

        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLLocomotion2";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLLocomotion3";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLLocomotion4";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        if (TryNavStateChange("RLFSMStateGetUp"))          return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateRLLocomotion"))   return "RLFSMStateRLLocomotion";
        if (TryNavStateChange("RLFSMStateRLLocomotion2"))  return "RLFSMStateRLLocomotion2";
        if (TryNavStateChange("RLFSMStateRLLocomotion3"))  return "RLFSMStateRLLocomotion3";
        if (TryNavStateChange("RLFSMStateRLLocomotion4"))  return "RLFSMStateRLLocomotion4";
        return state_name_;
    }

private:
    void CheckSafety()  // RLFSMStateRLLocomotion3
    {
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        // check robot attitude
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // check robot torque
        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        //  [w, x, y, z]
        float qw = imu.quaternion[0]; // x
        float qx = imu.quaternion[1]; // y
        float qy = imu.quaternion[2]; // z
        float qz = imu.quaternion[3]; // w
        
        // 检查四元数是否标准化
        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            // 标准化四元数
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        
        // 计算欧拉角 (航空zyx顺序，即yaw-pitch-roll)
        float roll, pitch, yaw;
        
        // 计算旋转矩阵元素
        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);
        
        // 从旋转矩阵计算欧拉角
        pitch = -asin(m20);
        
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            // 万向锁情况
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }
        
        // 转换为度
        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;
        
        // 检查姿态阈值
        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;
        
        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2) 
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }
        
        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }


};

class RLFSMStateRLLocomotion4 : public RLFSMState
{
public:
    RLFSMStateRLLocomotion4(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion4") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        rl.history_obs_buf.reset({0}, std::vector<float>());

        // read params from yaml
        rl.config_name = "robot_lab4";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();

        static int safety_counter = 0;
        safety_counter++;
        if(safety_counter % 10 == 0)
        {
            CheckSafety();
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            return "RLFSMStateCalfSwing";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            return "RLFSMStateClimbWall";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            return "RLFSMStateBridge";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLLocomotion2";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLLocomotion3";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLLocomotion4";
        }
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive"))        return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetDown"))        return "RLFSMStateGetDown";
        if (TryNavStateChange("RLFSMStateGetUp"))          return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))      return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))         return "RLFSMStateBridge";
        if (TryNavStateChange("RLFSMStateRLLocomotion"))   return "RLFSMStateRLLocomotion";
        if (TryNavStateChange("RLFSMStateRLLocomotion2"))  return "RLFSMStateRLLocomotion2";
        if (TryNavStateChange("RLFSMStateRLLocomotion3"))  return "RLFSMStateRLLocomotion3";
        if (TryNavStateChange("RLFSMStateRLLocomotion4"))  return "RLFSMStateRLLocomotion4";
        return state_name_;
    }

private:
    void CheckSafety()  // RLFSMStateRLLocomotion4
    {
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
        // check robot attitude
        if(IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // check robot torque
        if(IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal , switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;

        //  [w, x, y, z]
        float qw = imu.quaternion[0]; // x
        float qx = imu.quaternion[1]; // y
        float qy = imu.quaternion[2]; // z
        float qz = imu.quaternion[3]; // w
        
        // 检查四元数是否标准化
        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm != 0.0f && abs(norm - 1.0f) > 1e-6) {
            // 标准化四元数
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }
        
        // 计算欧拉角 (航空zyx顺序，即yaw-pitch-roll)
        float roll, pitch, yaw;
        
        // 计算旋转矩阵元素
        float m00 = 1.0f - 2.0f*(qy*qy + qz*qz);
        float m01 = 2.0f*(qx*qy - qw*qz);
        float m02 = 2.0f*(qx*qz + qw*qy);
        float m10 = 2.0f*(qx*qy + qw*qz);
        float m11 = 1.0f - 2.0f*(qx*qx + qz*qz);
        float m12 = 2.0f*(qy*qz - qw*qx);
        float m20 = 2.0f*(qx*qz - qw*qy);
        float m21 = 2.0f*(qy*qz + qw*qx);
        float m22 = 1.0f - 2.0f*(qx*qx + qy*qy);
        
        // 从旋转矩阵计算欧拉角
        pitch = -asin(m20);
        
        if (abs(cos(pitch)) > 1e-6) {
            roll = atan2(m21, m22);
            yaw = atan2(m10, m00);
        } else {
            // 万向锁情况
            roll = 0.0f;
            yaw = atan2(-m01, m11);
        }
        
        // 转换为度
        float roll_deg = roll * 180.0f / M_PI;
        float pitch_deg = pitch * 180.0f / M_PI;
        float yaw_deg = yaw * 180.0f / M_PI;
        
        // 检查姿态阈值
        float pitch_threshold = rl.params.Get<float>("pitch_threshold", 30.0f);
        float roll_threshold = rl.params.Get<float>("roll_threshold", 30.0f);

        bool attitude_abnormal = abs(pitch_deg) > pitch_threshold || abs(roll_deg) > roll_threshold;
        
        if (attitude_abnormal)
        {
            std::cout << LOGGER::WARNING << "Attitude abnormal detected! RPY: " << std::fixed << std::setprecision(2) 
                      << "Roll: " << roll_deg << "°, Pitch: " << pitch_deg << "°, Yaw: " << yaw_deg << "° "
                      << "(Thresholds: Roll ±" << roll_threshold << "°, Pitch ±" << pitch_threshold << "°)" << std::endl;
        }
        
        return attitude_abnormal;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& joints = fsm_state->motor_state;
        const auto& torque_limits = rl.params.Get<std::vector<float>>("torque_limits");

        for(size_t i = 0; i < joints.tau_est.size(); ++i)
        {
            if (abs(joints.tau_est[i]) > torque_limits[i])
            {
                return true;
            }
        }
        return false;
    }


};


class RLFSMStateClimbWall : public RLFSMState
{
public:
    RLFSMStateClimbWall(RL *rl) : RLFSMState(*rl, "RLFSMStateClimbWall") {}

    float step_progress_ = 0.0f;
    float hold_timer_ = 0.0f;
    int current_step_ = 0;
    bool step_done_ = false;
    std::vector<float> start_pos_;
    std::vector<float> sequence_;

    void Enter() override
    {
        step_progress_ = 0.0f;
        hold_timer_ = 0.0f;
        current_step_ = 0;
        step_done_ = false;

        sequence_ = rl.params.Get<std::vector<float>>("climb_wall_sequence");
        if (sequence_.size() < 14)
        {
            std::cout << LOGGER::WARNING << "ClimbWall: sequence too short, returning to Passive." << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // 从当前关节位置开始
        start_pos_ = fsm_state->motor_state.q;
        std::cout << LOGGER::NOTE << "ClimbWall: " << (sequence_.size() / 14) << " steps loaded." << std::endl;
    }

    void Run() override
    {
        if (sequence_.empty()) return;

        const float dt = rl.params.Get<float>("dt");
        const int step_offset = current_step_ * 14;

        if (static_cast<size_t>(step_offset + 13) >= sequence_.size())
        {
            // 序列结束
            bool loop = rl.params.Get<bool>("climb_wall_loop", false);
            if (loop)
            {
                current_step_ = 0;
                start_pos_ = fsm_state->motor_state.q;
                step_progress_ = 0.0f;
                step_done_ = false;
                std::cout << LOGGER::NOTE << "ClimbWall: looping." << std::endl;
            }
            else
            {
                std::cout << LOGGER::NOTE << "ClimbWall: sequence finished, staying." << std::endl;
                return;
            }
            return;
        }

        float duration = sequence_[step_offset];
        float hold = sequence_[step_offset + 1];

        // 构建目标姿态
        std::vector<float> target_pos(12);
        for (int i = 0; i < 12; ++i)
            target_pos[i] = sequence_[step_offset + 2 + i];

        if (!step_done_)
        {
            // 插值到目标姿态
            if (Interpolate(step_progress_, start_pos_, target_pos, duration, "ClimbWall step", true))
                return;
            step_done_ = true;
            hold_timer_ = 0.0f;
            std::cout << LOGGER::NOTE << "ClimbWall: step " << current_step_ << " done, holding " << hold << "s." << std::endl;
        }
        else
        {
            // 保持姿态
            hold_timer_ += dt;
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
            const int num_of_dofs = rl.params.Get<int>("num_of_dofs");
            for (int i = 0; i < num_of_dofs && i < 12; ++i)
            {
                fsm_command->motor_command.q[i]  = target_pos[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = kd[i];
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "ClimbWall: step " << current_step_
                      << " holding... " << std::fixed << std::setprecision(1)
                      << (hold - hold_timer_) << "s remaining" << std::flush;
            if (hold_timer_ >= hold)
            {
                std::cout << std::endl;
                current_step_++;
                start_pos_ = target_pos;
                step_progress_ = 0.0f;
                step_done_ = false;
            }
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
        {
            // 重新触发：重置序列从头播放
            current_step_ = 0;
            step_progress_ = 0.0f;
            step_done_ = false;
            start_pos_ = fsm_state->motor_state.q;
            std::cout << LOGGER::NOTE << "ClimbWall: re-triggered." << std::endl;
            return state_name_;
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
            return "RLFSMStateBridge";
        // 导航模式下的状态转换
        if (TryNavStateChange("RLFSMStatePassive"))    return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetUp"))      return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))  return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))     return "RLFSMStateBridge";
        return state_name_;
    }
};


class RLFSMStateBridge : public RLFSMState
{
public:
    RLFSMStateBridge(RL *rl) : RLFSMState(*rl, "RLFSMStateBridge") {}

    float step_progress_ = 0.0f;
    float hold_timer_ = 0.0f;
    int current_step_ = 0;
    bool step_done_ = false;
    std::vector<float> start_pos_;
    std::vector<float> sequence_;

    void Enter() override
    {
        step_progress_ = 0.0f;
        hold_timer_ = 0.0f;
        current_step_ = 0;
        step_done_ = false;

        sequence_ = rl.params.Get<std::vector<float>>("bridge_sequence");
        if (sequence_.size() < 14)
        {
            std::cout << LOGGER::WARNING << "Bridge: sequence too short, returning to Passive." << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        // 从当前关节位置开始
        start_pos_ = fsm_state->motor_state.q;
        std::cout << LOGGER::NOTE << "Bridge: " << (sequence_.size() / 14) << " steps loaded." << std::endl;
    }

    void Run() override
    {
        if (sequence_.empty()) return;

        const float dt = rl.params.Get<float>("dt");
        const int step_offset = current_step_ * 14;

        if (static_cast<size_t>(step_offset + 13) >= sequence_.size())
        {
            // 序列结束
            bool loop = rl.params.Get<bool>("bridge_loop", false);
            if (loop)
            {
                current_step_ = 0;
                start_pos_ = fsm_state->motor_state.q;
                step_progress_ = 0.0f;
                step_done_ = false;
                std::cout << LOGGER::NOTE << "Bridge: looping." << std::endl;
            }
            else
            {
                std::cout << LOGGER::NOTE << "Bridge: sequence finished, staying." << std::endl;
                return;
            }
            return;
        }

        float duration = sequence_[step_offset];
        float hold = sequence_[step_offset + 1];

        // 构建目标姿态
        std::vector<float> target_pos(12);
        for (int i = 0; i < 12; ++i)
            target_pos[i] = sequence_[step_offset + 2 + i];

        if (!step_done_)
        {
            // 插值到目标姿态
            if (Interpolate(step_progress_, start_pos_, target_pos, duration, "Bridge step", true))
                return;
            step_done_ = true;
            hold_timer_ = 0.0f;
            std::cout << LOGGER::NOTE << "Bridge: step " << current_step_ << " done, holding " << hold << "s." << std::endl;
        }
        else
        {
            // 保持姿态
            hold_timer_ += dt;
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
            const int num_of_dofs = rl.params.Get<int>("num_of_dofs");
            for (int i = 0; i < num_of_dofs && i < 12; ++i)
            {
                fsm_command->motor_command.q[i]  = target_pos[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = kd[i];
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Bridge: step " << current_step_
                      << " holding... " << std::fixed << std::setprecision(1)
                      << (hold - hold_timer_) << "s remaining" << std::flush;
            if (hold_timer_ >= hold)
            {
                std::cout << std::endl;
                current_step_++;
                start_pos_ = target_pos;
                step_progress_ = 0.0f;
                step_done_ = false;
            }
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
            return "RLFSMStateClimbWall";
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
        {
            // 重新触发：重置序列从头播放
            current_step_ = 0;
            step_progress_ = 0.0f;
            step_done_ = false;
            start_pos_ = fsm_state->motor_state.q;
            std::cout << LOGGER::NOTE << "Bridge: re-triggered." << std::endl;
            return state_name_;
        }
        // 导航模式下的状态转换
        if (TryNavStateChange("RLFSMStatePassive"))    return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetUp"))      return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))  return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))     return "RLFSMStateBridge";
        return state_name_;
    }
};


class RLFSMStateSquat : public RLFSMState
{
public:
    RLFSMStateSquat(RL *rl) : RLFSMState(*rl, "RLFSMStateSquat") {}

    enum Phase { PHASE_SQUAT_DOWN = 0, PHASE_HOLD = 1, PHASE_STAND_UP = 2 };

    int phase_ = PHASE_SQUAT_DOWN;
    float phase_progress_ = 0.0f;
    float hold_timer_ = 0.0f;
    float squat_down_duration_ = 1.0f;
    float hold_duration_ = 3.0f;
    float stand_up_duration_ = 1.5f;
    float squat_angle_ = 0.3f;        // 大腿旋转角度 (rad)
    float squat_calf_angle_ = 0.3f;   // 小腿旋转角度 (rad)，默认 = squat_angle
    std::vector<float> start_pos_;
    std::vector<float> target_pos_;

    // 关节索引: {FL, FR, RL, RR}
    static constexpr int HIP_IDX[4]   = {3, 0, 9, 6};
    static constexpr int THIGH_IDX[4] = {4, 1, 10, 7};
    static constexpr int CALF_IDX[4]  = {5, 2, 11, 8};

    void Enter() override
    {
        phase_ = PHASE_SQUAT_DOWN;
        phase_progress_ = 0.0f;
        hold_timer_ = 0.0f;

        squat_down_duration_ = rl.params.Get<float>("squat_down_duration", 2.0f);
        hold_duration_       = rl.params.Get<float>("squat_hold_duration", 2.0f);
        stand_up_duration_   = rl.params.Get<float>("squat_stand_up_duration", 2.0f);
        squat_angle_         = rl.params.Get<float>("squat_angle", 0.3f);
        squat_calf_angle_    = rl.params.Get<float>("squat_calf_angle", squat_angle_);

        auto dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        const int num_of_dofs = rl.params.Get<int>("num_of_dofs");

        start_pos_  = dof_pos;
        target_pos_ = dof_pos;
        for (int leg = 0; leg < 4; ++leg)
        {
            int ti = THIGH_IDX[leg];
            int ci = CALF_IDX[leg];
            float sign = (dof_pos[ti] >= 0.0f) ? 1.0f : -1.0f;
            if (ti < num_of_dofs && ti < static_cast<int>(target_pos_.size()))
                target_pos_[ti] += sign * squat_angle_;
            if (ci < num_of_dofs && ci < static_cast<int>(target_pos_.size()))
                target_pos_[ci] -= sign * squat_calf_angle_;  // 小腿反向
        }

        std::cout << LOGGER::NOTE << "Entered squat state: angle=" << squat_angle_
                  << "rad, down=" << squat_down_duration_ << "s, hold=" << hold_duration_
                  << "s, up=" << stand_up_duration_ << "s" << std::endl;
    }

    void Run() override
    {
        static int safety_counter = 0;
        safety_counter++;
        if (safety_counter % 10 == 0) { CheckSafety(); }

        const float dt = rl.params.Get<float>("dt");

        switch (phase_)
        {
        case PHASE_SQUAT_DOWN:
            if (Interpolate(phase_progress_, start_pos_, target_pos_, squat_down_duration_, "Squat down", true))
                return;
            phase_ = PHASE_HOLD;
            hold_timer_ = 0.0f;
            std::cout << LOGGER::NOTE << "Squat hold begin, holding for " << hold_duration_ << "s" << std::endl;
            break;

        case PHASE_HOLD:
        {
            hold_timer_ += dt;
            // 保持下蹲姿态
            auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
            auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
            const int num_of_dofs = rl.params.Get<int>("num_of_dofs");
            for (int i = 0; i < num_of_dofs && i < static_cast<int>(target_pos_.size()); ++i)
            {
                fsm_command->motor_command.q[i]  = target_pos_[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = kp[i];
                fsm_command->motor_command.kd[i] = kd[i];
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Squat holding... "
                      << std::fixed << std::setprecision(1) << (hold_duration_ - hold_timer_) << "s remaining" << std::flush;
            if (hold_timer_ >= hold_duration_)
            {
                phase_ = PHASE_STAND_UP;
                phase_progress_ = 0.0f;
                std::cout << std::endl << LOGGER::NOTE << "Standing up..." << std::endl;
            }
            break;
        }

        case PHASE_STAND_UP:
            if (Interpolate(phase_progress_, target_pos_, start_pos_, stand_up_duration_, "Stand up", true))
                return;
            std::cout << LOGGER::NOTE << "Squat sequence finished, standing." << std::endl;
            rl.fsm.RequestStateChange("RLFSMStateGetUp");
            break;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::RT_DPadUp)
            return "RLFSMStateClimbWall";
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::RT_DPadDown)
            return "RLFSMStateBridge";
        if (rl.control.current_keyboard == Input::Keyboard::Num5)
            return "RLFSMStateCalfSwing";
        // 导航模式下的状态转换（排除 CalfSwing）
        if (TryNavStateChange("RLFSMStatePassive")) return "RLFSMStatePassive";
        if (TryNavStateChange("RLFSMStateGetUp"))   return "RLFSMStateGetUp";
        if (TryNavStateChange("RLFSMStateClimbWall"))   return "RLFSMStateClimbWall";
        if (TryNavStateChange("RLFSMStateBridge"))      return "RLFSMStateBridge";
        return state_name_;
    }

private:
    // RLFSMStateSquat
    void CheckSafety()
    {
        if(AreMotorsOffline())
        {
            std::cout << LOGGER::WARNING << "Motors offline/stalled detected, switching to passive state" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive"); return;
        }
        if (IsRobotAttitudeAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot attitude abnormal, switching to passive" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive"); return;
        }
        if (IsJointTorqueAbnormal())
        {
            std::cout << LOGGER::WARNING << "Robot torque abnormal, switching to passive" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive"); return;
        }
    }

    bool IsRobotAttitudeAbnormal()
    {
        const auto& imu = fsm_state->imu;
        float qw = imu.quaternion[0], qx = imu.quaternion[1], qy = imu.quaternion[2], qz = imu.quaternion[3];
        float n = sqrt(qw*qw+qx*qx+qy*qy+qz*qz);
        if (n > 0 && abs(n-1)>1e-6) { qw/=n; qx/=n; qy/=n; qz/=n; }
        float m20=2*(qx*qz-qw*qy), m21=2*(qy*qz+qw*qx), m22=1-2*(qx*qx+qy*qy);
        float m10=2*(qx*qy+qw*qz), m00=1-2*(qy*qy+qz*qz), m01=2*(qx*qy-qw*qz);
        float pitch=-asin(m20), roll;
        if (abs(cos(pitch))>1e-6) roll=atan2(m21,m22); else roll=0;
        float rd=roll*180/M_PI, pd=pitch*180/M_PI;
        float pt=rl.params.Get<float>("pitch_threshold",30), rt=rl.params.Get<float>("roll_threshold",30);
        if (abs(pd)>pt||abs(rd)>rt)
            std::cout<<LOGGER::WARNING<<"Attitude abnormal! Roll:"<<rd<<"° Pitch:"<<pd<<"° (Thr:"<<rt<<"/"<<pt<<")"<<std::endl;
        return abs(pd)>pt||abs(rd)>rt;
    }

    bool IsJointTorqueAbnormal()
    {
        const auto& j=fsm_state->motor_state;
        const auto& tl=rl.params.Get<std::vector<float>>("torque_limits");
        for(size_t i=0;i<j.tau_est.size();++i) if(abs(j.tau_est[i])>tl[i]) return true;
        return false;
    }
};

} // namespace my_dog_fsm

class MY_DOG_FSMFactory : public FSMFactory
{
public:
    MY_DOG_FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<my_dog_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<my_dog_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<my_dog_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateCalfSwing")
            return std::make_shared<my_dog_fsm::RLFSMStateCalfSwing>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<my_dog_fsm::RLFSMStateRLLocomotion>(rl);
        else if (state_name == "RLFSMStateRLLocomotion2")
            return std::make_shared<my_dog_fsm::RLFSMStateRLLocomotion2>(rl);
        else if (state_name == "RLFSMStateRLLocomotion3")
            return std::make_shared<my_dog_fsm::RLFSMStateRLLocomotion3>(rl);
        else if (state_name == "RLFSMStateRLLocomotion4")
            return std::make_shared<my_dog_fsm::RLFSMStateRLLocomotion4>(rl);
        else if (state_name == "RLFSMStateClimbWall")
            return std::make_shared<my_dog_fsm::RLFSMStateClimbWall>(rl);
        else if (state_name == "RLFSMStateBridge")
            return std::make_shared<my_dog_fsm::RLFSMStateBridge>(rl);
        else if (state_name == "RLFSMStateSquat")
            return std::make_shared<my_dog_fsm::RLFSMStateSquat>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "my_dog"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateCalfSwing",
            "RLFSMStateRLLocomotion",
            "RLFSMStateRLLocomotion2",
            "RLFSMStateRLLocomotion3",
            "RLFSMStateRLLocomotion4",
            "RLFSMStateClimbWall",
            "RLFSMStateBridge",
            "RLFSMStateSquat"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(MY_DOG_FSMFactory, "RLFSMStatePassive")

#endif // MY_DOG_FSM_HPP