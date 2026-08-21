#include "rl_real_my_dog.hpp"

RL_Real::RL_Real(int argc, char **argv) // RL_Real类的构造函数
{
    // 这里的实现和go2类似，主要区别在于my_dog的通信接口和状态反馈不同，需要自己实现
    // 读yaml参数，初始化状态机，创建通信接口等
    // 具体实现可以参考rl_real_go2.cpp中的RL_Real构造函数

#if defined(USE_ROS1) && defined(USE_ROS)
    ros::NodeHandle nh;
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Real::JoyCallback, this);
#elif defined(USE_ROS2)
    // 创建rl_real_node节点，订阅cmd/vel话题，通信质量，消息种类，回调函数
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>("/red_standard_robot1/cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                                         { this->CmdvelCallback(msg); });
    this->nav_state_subscriber = ros2_node->create_subscription<std_msgs::msg::String>("/fsm_state_request", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::String::SharedPtr msg)
                                                                                         { this->NavStateCallback(msg); });
    this->nav_mode_subscriber = ros2_node->create_subscription<std_msgs::msg::String>("/nav_mode", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::String::SharedPtr msg)
                                                                                         { this->NavModeCallback(msg); });
    this->joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Joy::SharedPtr msg)
                                                                                 { this->JoyCallback(msg); });
    if (!ros2_node)
    {
        std::cerr << "Failed to create ROS2 node" << std::endl;
        throw std::runtime_error("Failed to create ROS2 node");
    }
#endif

    // read params from yaml
    this->ang_vel_axis = "body";                   // 使用机体坐标系
    this->robot_name = "my_dog";                   // 名称是my_dog
    this->ReadYaml(this->robot_name, "base.yaml"); // 读取参数文件
    // auto load FSM by robot_name
    // 状态机管理器检查是否支持该机器人(my_dog)
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        // 创建状态机实例指针
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->my_dog_robot_command.motor_command.resize(this->params.Get<int>("num_of_dofs"));
    this->my_dog_motor_states.motor_state.resize(this->params.Get<int>("num_of_dofs"));

    joint_names_ =
        {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

    this->InitLowCmd();                                       // 初始化底层命令,将准备发送出去的各个电机模式设置为停止，且其他参数统统设置为0
    this->InitJointNum(this->params.Get<int>("num_of_dofs")); // 初始化关节数量,已经在rl_sdk实现
    this->InitOutputs();                                      // 初始化输出，已经在rl_sdk实现，根据关节数量，腾空间给扭矩和速度并设置为0,将位置设置为默认位姿
    this->InitControl();                                      // 初始化控制值：x,y,yaw，已经在rl_sdk实现

    // 创建底层命令发布者
    try
    {
        this->my_dog_robot_command_publisher = ros2_node->create_publisher<robot_msgs::msg::RobotCommand>(TOPIC_ROBOT_CMD, rclcpp::SystemDefaultsQoS());
        if (!this->my_dog_robot_command_publisher)
        {
            std::cerr << "Failed to create ROS2 publisher for robot command" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating publisher: " << e.what() << std::endl;
    }

    // 从my_dog_state话题订阅状态反馈
    try
    {
        this->my_dog_motor_states_subscriber = ros2_node->create_subscription<robot_msgs::msg::MotorStates>(TOPIC_MOTOR_STATES, rclcpp::SystemDefaultsQoS(), [this](const robot_msgs::msg::MotorStates::SharedPtr msg)
                                                                                                            { this->MotorStatesCallback(msg); });
        if (!this->my_dog_motor_states_subscriber)
        {
            std::cerr << "Failed to create ROS2 subscriber for motor states" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating subscriber: " << e.what() << std::endl;
    }

    try
    {
        this->my_dog_imu_state_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(TOPIC_IMU_STATES, rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg)
                                                                                                  { this->IMUStateCallback(msg); });
        if (!this->my_dog_imu_state_subscriber)
        {
            std::cerr << "Failed to create ROS2 subscriber for imu state" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating subscriber: " << e.what() << std::endl;
    }

    // 创建关节状态发布者（提供给foxglove控制urdf进行可视化）
    try
    {
        this->joint_states_publisher = ros2_node->create_publisher<sensor_msgs::msg::JointState>(TOPIC_JOINT_STATES, rclcpp::SystemDefaultsQoS());
        if (!this->joint_states_publisher)
        {
            std::cerr << "Failed to create ROS2 publisher for joint states" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating publisher: " << e.what() << std::endl;
    }

    // 创建 /body_velocity 话题发布者
    try
    {
        this->body_velocity_publisher_ = ros2_node->create_publisher<geometry_msgs::msg::Twist>(
            "/body_velocity", rclcpp::SystemDefaultsQoS());
        if (!this->body_velocity_publisher_)
        {
            std::cerr << "Failed to create ROS2 publisher for body_velocity" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating body_velocity publisher: " << e.what() << std::endl;
    }

    // 创建 FSM 状态发布者（供外部导航系统感知当前 FSM 状态）
    try
    {
        this->fsm_state_publisher_ = ros2_node->create_publisher<std_msgs::msg::String>(
            "/fsm_state", rclcpp::SystemDefaultsQoS());
        if (!this->fsm_state_publisher_)
        {
            std::cerr << "Failed to create ROS2 publisher for /fsm_state" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception creating /fsm_state publisher: " << e.what() << std::endl;
    }
    // 初始化动作切换客户端

    // 关闭运动控制切换的相关服务

    // RC 遥控器初始化 (YAML: rc_enabled, rc_port, rc_baud)
    InitRC();

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    if (this->rc_enabled_)
    {
        this->loop_rc = std::make_shared<LoopFunc>("rc_input", 0.01, std::bind(&RL_Real::RCInterface, this));
    }
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
    if (this->rc_enabled_ && this->loop_rc)
    {
        this->loop_rc->start();
    }
#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos = std::vector<std::vector<float>>(this->params.Get<int>("num_of_dofs"), std::vector<float>(this->plot_size, 0.0f));
    this->plot_target_joint_pos = std::vector<std::vector<float>>(this->params.Get<int>("num_of_dofs"), std::vector<float>(this->plot_size, 0.0f));
    for (auto &vector : this->plot_real_joint_pos)
    {
        vector = std::vector<float>(this->plot_size, 0);
    }
    for (auto &vector : this->plot_target_joint_pos)
    {
        vector = std::vector<float>(this->plot_size, 0);
    }
    this->loop_plot = std::make_shared<LoopFunc>("plot", 0.1f, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
//#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
//#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    if (this->loop_rc) this->loop_rc->shutdown();
    this->rc_reader_.stop();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif

#if defined USE_ROS2
    if (this->my_dog_robot_command_publisher)
    {
        this->my_dog_robot_command_publisher.reset();
    }
    if (this->my_dog_motor_states_subscriber)
    {
        this->my_dog_motor_states_subscriber.reset();
    }
    if (this->my_dog_imu_state_subscriber)
    {
        this->my_dog_imu_state_subscriber.reset();
    }
    if (this->cmd_vel_subscriber)
    {
        this->cmd_vel_subscriber.reset();
    }
    if (this->joy_subscriber)
    {
        this->joy_subscriber.reset();
    }
    if (this->joint_states_publisher)
    {
        this->joint_states_publisher.reset();
    }
    if (this->ros2_node)
    {
        this->ros2_node.reset();
    }
    if (this->body_velocity_publisher_)
    {
        this->body_velocity_publisher_.reset();
    }
#endif
}

// ============================================================
// RC 遥控器 (ET08A W.BUS) 接口
// ============================================================
void RL_Real::InitRC()
{
    this->rc_enabled_ = this->params.Get<bool>("rc_enabled", false);
    if (!this->rc_enabled_)
    {
        std::cout << LOGGER::NOTE << "[RC] RC disabled (rc_enabled=false in YAML). Using ROS /joy or keyboard." << std::endl;
        return;
    }

    this->rc_port_ = this->params.Get<std::string>("rc_port", "/dev/ttyUSB0");
    this->rc_baud_ = this->params.Get<int>("rc_baud", 100000);

    // 先配置通道映射（即使串口尚未打开）
    int ch_roll    = this->params.Get<int>("rc_ch_roll",    0);
    int ch_pitch   = this->params.Get<int>("rc_ch_pitch",   1);
    int ch_throttle= this->params.Get<int>("rc_ch_throttle",2);
    int ch_yaw     = this->params.Get<int>("rc_ch_yaw",     3);
    int ch_sa      = this->params.Get<int>("rc_ch_sa",      4);
    int ch_sb      = this->params.Get<int>("rc_ch_sb",      5);
    int ch_sc      = this->params.Get<int>("rc_ch_sc",      6);
    int ch_rd      = this->params.Get<int>("rc_ch_rd",      7);
    rc_mapper_.setChannelMap(ch_roll, ch_pitch, ch_throttle, ch_yaw,
                             ch_sa, ch_sb, ch_sc, ch_rd);

    std::cout << LOGGER::NOTE << "[RC] Mapping:" << std::endl;
    std::cout << LOGGER::NOTE << "  Sticks: CH1→yaw  CH2→x  CH4→y" << std::endl;
    std::cout << LOGGER::NOTE << "  Knob:   CH" << (ch_rd+1) << "→RD [0~1], max→poweroff" << std::endl;
    std::cout << LOGGER::NOTE << "  SA (CH" << (ch_sa+1) << "): HIGH→RL1  LOW→RL2" << std::endl;
    std::cout << LOGGER::NOTE << "  SB (CH" << (ch_sb+1) << "): HIGH→MotorEnable  LOW→MotorDisable" << std::endl;
    std::cout << LOGGER::NOTE << "  SC (CH" << (ch_sc+1) << "): LOW→Passive  HIGH→GetUp" << std::endl;

    // 尝试打开串口（失败不放弃，RCInterface 会持续重试）
    std::cout << LOGGER::NOTE << "[RC] Opening " << rc_port_ << " @ " << rc_baud_ << " baud..." << std::endl;

    if (rc_reader_.open(rc_port_, rc_baud_))
    {
        rc_reader_.start();
        std::cout << LOGGER::NOTE << "[RC] Serial port opened, background reader started." << std::endl;
    }
    else
    {
        std::cout << LOGGER::WARNING << "[RC] Failed to open " << rc_port_
                  << ". Will retry in background. Check: "
                  << "sudo chmod 666 " << rc_port_
                  << " or add user to dialout group." << std::endl;
        // 不设 rc_enabled_=false, RCInterface 会持续重试
    }
}

void RL_Real::RCInterface()
{
    if (!rc_enabled_)
        return;

    // 串口未打开 → 每 2 秒重试一次
    if (!rc_reader_.isRunning())
    {
        int retry_interval = static_cast<int>(2.0f / this->params.Get<float>("dt", 0.001f));
        if (this->motiontime - rc_last_retry_ > retry_interval)
        {
            rc_last_retry_ = this->motiontime;
            if (rc_reader_.open(rc_port_, rc_baud_))
            {
                rc_reader_.start();
                std::cout << LOGGER::NOTE << "[RC] Serial port reconnected on " << rc_port_ << "!" << std::endl;
            }
        }
        return;
    }

    sbus::Frame frame;
    if (!rc_reader_.getLatest(frame))
        return;

    Input::Gamepad gp = Input::Gamepad::None;
    float x = 0.0f, y = 0.0f, yaw = 0.0f, rd = 0.0f;

    if (rc_mapper_.process(frame, gp, x, y, yaw, rd))
    {
        if (gp != Input::Gamepad::None)
            this->control.SetGamepad(gp);
        this->control.x   = x;
        this->control.y   = y;
        this->control.yaw = yaw;
        this->rc_rd_      = rd;

        // 诊断输出 (每秒一次)
        static int diag_cnt = 0;
        diag_cnt++;
        if (diag_cnt % 100 == 0)
        {
            std::cout << LOGGER::INFO
                      << "[RC] gp=" << static_cast<int>(gp)
                      << " x=" << std::fixed << std::setprecision(2) << x
                      << " y=" << y << " yaw=" << yaw
                      << " rd=" << rd << std::endl;
        }

        // RD 旋钮关机: 逆时针拧到底 (>1600 原始值, rd > 0.93), 只触发一次
        if (!rc_poweroff_triggered_ && rd > 0.93f)
        {
            rc_poweroff_triggered_ = true;
            std::cout << LOGGER::WARNING << "[RC] RD knob at maximum (rd=" << rd
                      << "), executing poweroff.sh..." << std::endl;
            int ret = system("bash $HOME/rl_sar/src/rl_sar/poweroff.sh &");
            if (ret != 0)
                std::cout << LOGGER::ERROR << "[RC] poweroff.sh failed: " << ret << std::endl;
        }
    }
    // failsafe/frame_lost → 不更新 control, 保持上一帧指令
}

void RL_Real::InitLowCmd()
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->my_dog_robot_command.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = 0;
        this->my_dog_robot_command.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = 0;
        this->my_dog_robot_command.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = 0;
        this->my_dog_robot_command.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = 0;
        this->my_dog_robot_command.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = 0;
    }
}

// 获取机器人状态(当前的运动控制(x,y,yaw)指令和imu+电机状态)
void RL_Real::GetState(RobotState<float> *state)
{
    {
        {
                std::lock_guard<std::mutex> lock(this->imu_state_mutex_);

                state->imu.quaternion[0] = this->my_dog_imu_state.orientation.w;
                state->imu.quaternion[1] = this->my_dog_imu_state.orientation.x;
                state->imu.quaternion[2] = this->my_dog_imu_state.orientation.y;
                state->imu.quaternion[3] = this->my_dog_imu_state.orientation.z;

                state->imu.gyroscope[0] = this->my_dog_imu_state.angular_velocity.x;
                state->imu.gyroscope[1] = this->my_dog_imu_state.angular_velocity.y;
                state->imu.gyroscope[2] = this->my_dog_imu_state.angular_velocity.z;

                state->imu.accelerometer[0] = this->my_dog_imu_state.linear_acceleration.x;
                state->imu.accelerometer[1] = this->my_dog_imu_state.linear_acceleration.y;
                state->imu.accelerometer[2] = this->my_dog_imu_state.linear_acceleration.z;
            }

        {
            std::lock_guard<std::mutex> lock(this->motor_states_mutex_);
            for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
            {
                state->motor_state.q[i] = this->my_dog_motor_states.motor_state[i].q;
                state->motor_state.dq[i] = this->my_dog_motor_states.motor_state[i].dq;
                state->motor_state.ddq[i] = this->my_dog_motor_states.motor_state[i].ddq;
                state->motor_state.tau_est[i] = this->my_dog_motor_states.motor_state[i].tau_est;
                state->motor_state.cur[i] = this->my_dog_motor_states.motor_state[i].cur;
            }
        }
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // 这里需要实现将机器人命令发送给my_dog的代码，具体实现会根据my_dog的通信协议和接口定义有所不同，需要根据实际情况进行调整
    // 例如，将command中的关节命令转换为LowCmd消息，并通过lowcmd_publisher发送出去
    // 具体实现可以参考rl_real_go2.cpp中的RobotControl函数，以及RobotCommand结构体的定义

    {
        std::lock_guard<std::mutex> lock(this->command_mutex_);

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            int mapped_idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
            this->my_dog_robot_command.motor_command[mapped_idx].q = command->motor_command.q[i];
            this->my_dog_robot_command.motor_command[mapped_idx].dq = command->motor_command.dq[i];
            this->my_dog_robot_command.motor_command[mapped_idx].kp = command->motor_command.kp[i];
            this->my_dog_robot_command.motor_command[mapped_idx].kd = command->motor_command.kd[i];
            this->my_dog_robot_command.motor_command[mapped_idx].tau = command->motor_command.tau[i];
        }
    }

    if (this->my_dog_robot_command_publisher)
    {
        this->my_dog_robot_command_publisher->publish(this->my_dog_robot_command);
    }
    else
    {
        std::cerr << "Failed to publish robot command" << std::endl;
    }
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);
    this->StateController(&this->robot_state, &this->robot_command);

    // 发布 FSM 状态变化（供外部导航系统感知当前状态）
    {
        std::string current_fsm = this->fsm.GetCurrentStateName();
        if (!current_fsm.empty() && current_fsm != this->last_fsm_state_)
        {
            this->last_fsm_state_ = current_fsm;
            auto msg = std_msgs::msg::String();
            msg.data = current_fsm;
            this->fsm_state_publisher_->publish(msg);
        }
    }

    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
}

void RL_Real::RunModel()
{
    // 这里需要实现模型推理的代码，主要包括从历史观测数据缓冲区中获取当前观测数据，调用Forward函数进行推理，并将结果存储到output_dof_tau等变量中
    // 具体实现可以参考rl_real_go2.cpp中的RunModel函数，以及Forward函数的定义
    if (this->rl_init_done)
    {
        // 执行模型推理
        {
            std::lock_guard<std::mutex> lock(this->robot_state_mutex_);

            this->episode_length_buf += 1;
            this->obs.ang_vel = this->robot_state.imu.gyroscope;
            this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
            if (this->control.navigation_mode)
            {
                this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
            }

#endif
            this->obs.base_quat = this->robot_state.imu.quaternion;
            this->obs.dof_pos = this->robot_state.motor_state.q;
            this->obs.dof_vel = this->robot_state.motor_state.dq;
        }
        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        this->PublishJointStates();

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    // 这里需要实现模型推理的具体代码，主要包括将当前观测数据传递给模型进行推理，并返回动作结果
    // 具体实现可以参考rl_real_go2.cpp中的Forward函数，以及InferenceRuntime::Model接口的定义
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being accessed by another thread or reinitialized,using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observation_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observation_history"));
        actions = this->model->forward({clamped_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    // 这里需要实现绘图的代码，主要包括将当前的关节位置和目标关节位置存储到plot_real_joint_pos和plot_target_joint_pos中，并使用matplotlibcpp进行绘图
    // 具体实现可以参考rl_real_go2.cpp中的Plot函数，以及matplotlibcpp的使用方法
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); i++)
    {
        // 绘图逻辑
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state.motor_state.q[i]);
        this->plot_target_joint_pos[i].push_back(this->robot_command.motor_command.q[i]);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i]);
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i]);
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

// uint32_t RL_Real::QueryMotionStatus()
//{
//  这里需要实现查询my_dog当前运动状态的代码，具体实现会根据my_dog的通信协议和接口定义有所不同，需要根据实际情况进行调整
//  例如，通过MotionSwitcherClient发送查询请求，并解析返回结果来判断当前是否处于运动控制相关的模式
//}

// std::string RL_Real::GetMotionStatusString(uint32_t status)
//{
//  这里需要实现将my_dog的运动状态转换为字符串的代码，具体实现会根据my_dog的通信协议和接口定义有所不同，需要根据实际情况进行调整
//  例如，根据查询到的状态值返回对应的字符串描述，如"Idle", "Walking", "Running"等
//}

void RL_Real::MotorStatesCallback(const robot_msgs::msg::MotorStates::SharedPtr msg)
{
    // 这里需要实现处理my_dog MotorStates消息的代码，主要包括从消息中提取状态信息，并更新机器人状态和历史观测数据缓冲区
    // 具体实现可以参考rl_real_go2.cpp中的MotorsStateCallback函数，以及MyDogMotorStatesMessage的定义

    if (msg->motor_state.size() != this->params.Get<int>("num_of_dofs"))
    {
        std::cout << LOGGER::WARNING << "Motor state size mismatch: expected "
                  << this->params.Get<int>("num_of_dofs") << ", got "
                  << msg->motor_state.size() << std::endl;
        return;
    }

    {
        // 加锁保护状态更新
        std::lock_guard<std::mutex> lock(this->motor_states_mutex_);
        this->my_dog_motor_states = *msg;
    }
}

void RL_Real::IMUStateCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 处理my_dog IMU数据消息，从消息中提取状态信息，并更新机器人状态
    {
        // 加锁保护状态更新
        std::lock_guard<std::mutex> lock(this->imu_state_mutex_);
        this->my_dog_imu_state = *msg;
    }

    // ================== 新增：发布机体速度 ==================
    if (this->body_velocity_publisher_)
    {
        // 角速度直接使用 IMU gyro
        this->body_velocity_msg_.angular.x = msg->angular_velocity.x;
        this->body_velocity_msg_.angular.y = msg->angular_velocity.y;
        this->body_velocity_msg_.angular.z = msg->angular_velocity.z;

       
        static rclcpp::Time last_time;
        static bool first_time = true;    //用于标志初始化
        static geometry_msgs::msg::Vector3 linear_vel;
        linear_vel.x = 0.0;
        linear_vel.y = 0.0;
        linear_vel.z = 0.0;


        rclcpp::Time current_time = this->ros2_node->get_clock()->now();

        if (first_time)
        {
            // 第一次只初始化时间，不进行积分
            last_time = current_time;
            first_time = false;

            linear_vel.x = 0.0;
            linear_vel.y = 0.0;
            linear_vel.z = 0.0;
        }
        else
        {
            double dt = (current_time - last_time).seconds();
            last_time = current_time;
            linear_vel.x += msg->linear_acceleration.x * dt;
            linear_vel.y += msg->linear_acceleration.y * dt;
            linear_vel.z += (msg->linear_acceleration.z - 9.81) * dt;  // 减去重力
        }

        this->body_velocity_msg_.linear.x = linear_vel.x;
        this->body_velocity_msg_.linear.y = linear_vel.y;
        this->body_velocity_msg_.linear.z = linear_vel.z;

        this->body_velocity_publisher_->publish(this->body_velocity_msg_);
    }
    // =======================================================
}


void RL_Real::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}

void RL_Real::NavStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
    this->control.nav_requested_state = msg->data;
    std::cout << LOGGER::INFO << "[Nav] Requested FSM state: " << msg->data << std::endl;
}

void RL_Real::NavModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string& cmd = msg->data;
    if (cmd == "ON")
    {
        this->control.navigation_mode = true;
        std::cout << LOGGER::INFO << "[NavMode] Navigation mode: ON" << std::endl;
    }
    else if (cmd == "OFF")
    {
        this->control.navigation_mode = false;
        std::cout << LOGGER::INFO << "[NavMode] Navigation mode: OFF" << std::endl;
    }
    else if (cmd == "TOGGLE")
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << LOGGER::INFO << "[NavMode] Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
    else
    {
        std::cout << LOGGER::WARNING << "[NavMode] Unknown command: " << cmd << " (use ON/OFF/TOGGLE)" << std::endl;
    }
}

void RL_Real::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons.size() < 11 || msg->axes.size() < 8)
    {
        std::cerr << LOGGER::WARNING << "Joy message size mismatch: buttons=" << msg->buttons.size()
                  << ", axes=" << msg->axes.size() << std::endl;
        return;
    }

    this->joy_msg = *msg;

    if (this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4])
        this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5])
        this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.axes[2] < -0.2 && this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::LT_B);
    if (this->joy_msg.axes[5] < -0.2 && this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::RT_DPadUp);
    if (this->joy_msg.axes[5] < -0.2 && this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::RT_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5])
        this->control.SetGamepad(Input::Gamepad::LB_RB);
    if (this->joy_msg.buttons[6] && this->joy_msg.buttons[7])
        this->control.SetGamepad(Input::Gamepad::Back_Start);

    this->control.x = this->joy_msg.axes[1];
    this->control.y = this->joy_msg.axes[0];
    this->control.yaw = this->joy_msg.axes[3];
}

void RL_Real::PublishJointStates()
{
    this->joint_states_msg.header.stamp = this->ros2_node->get_clock()->now();
    this->joint_states_msg.header.frame_id = "base";
    this->joint_states_msg.name = this->joint_names_;
    this->joint_states_msg.position.resize(this->joint_names_.size());
    this->joint_states_msg.velocity.resize(this->joint_names_.size());
    this->joint_states_msg.effort.resize(this->joint_names_.size());
    for (int i = 0; i < this->joint_names_.size(); i++)
    {
        this->joint_states_msg.position[i] = this->my_dog_motor_states.motor_state[i].q;
        this->joint_states_msg.velocity[i] = this->my_dog_motor_states.motor_state[i].dq;
        this->joint_states_msg.effort[i] = this->my_dog_motor_states.motor_state[i].tau_est;
    }
    this->joint_states_publisher->publish(this->joint_states_msg);
}

// void RL_Real::JoystickHandler(const MyDogJoystickMessage &msg)
// {
// 这里需要实现处理my_dog Joystick消息的代码，主要包括从消息中提取遥控器输入信息，并更新Control结构体中的相关变量
// 具体实现可以参考rl_real_go2.cpp中的JoystickHandler函数，以及MyDogJoystickMessage的定义
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
}