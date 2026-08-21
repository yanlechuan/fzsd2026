/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"

RL_Sim::RL_Sim(int argc, char **argv)
{
#if defined(USE_ROS1)
    this->ang_vel_axis = "world";
    ros::NodeHandle nh;
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    nh.param<std::string>("robot_name", this->robot_name, "");
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_sim_node");
    this->ang_vel_axis = "body";
    this->ros_namespace = ros2_node->get_namespace();
    // get params from param_node
    param_client = ros2_node->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(ros2_node->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }
#endif

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
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

    this->is_simulation = true;

    // init robot
#if defined(USE_ROS1)
    this->joint_publishers_commands.resize(this->params.Get<int>("num_of_dofs"));
#elif defined(USE_ROS2)
    this->robot_command_publisher_msg.motor_command.resize(this->params.Get<int>("num_of_dofs"));
    this->robot_state_subscriber_msg.motor_state.resize(this->params.Get<int>("num_of_dofs"));
#endif
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();

#if defined(USE_ROS1)
    auto joint_controller_names_vec = this->params.Get<std::vector<std::string>>("joint_controller_names"); // avoid dangling reference
    this->StartJointController(this->ros_namespace, joint_controller_names_vec);
    // publisher
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/command";
        this->joint_publishers[joint_controller_name] =
            nh.advertise<robot_msgs::MotorCommand>(topic_name, 10);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Sim::CmdvelCallback, this);
    this->joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &RL_Sim::JoyCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, &RL_Sim::ModelStatesCallback, this);
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        const std::string &joint_controller_name = joint_controller_names_vec[i];
        const std::string topic_name = this->ros_namespace + joint_controller_name + "/state";
        this->joint_subscribers[joint_controller_name] =
            nh.subscribe<robot_msgs::MotorState>(topic_name, 10,
                                                 [this, joint_controller_name](const robot_msgs::MotorState::ConstPtr &msg)
                                                 {
                                                     this->JointStatesCallback(msg, joint_controller_name);
                                                 });
        this->joint_positions[joint_controller_name] = 0.0f;
        this->joint_velocities[joint_controller_name] = 0.0f;
        this->joint_efforts[joint_controller_name] = 0.0f;
    }

    // service
    nh.param<std::string>("gazebo_model_name", this->gazebo_model_name, "");
    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    this->gazebo_reset_world_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
#elif defined(USE_ROS2)
    this->StartJointController(this->ros_namespace, this->params.Get<std::vector<std::string>>("joint_names"));
    // publisher
    this->robot_command_publisher = ros2_node->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        { this->CmdvelCallback(msg); });
    this->nav_state_subscriber = ros2_node->create_subscription<std_msgs::msg::String>(
        "/fsm_state_request", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        { this->NavStateCallback(msg); });
    this->nav_mode_subscriber = ros2_node->create_subscription<std_msgs::msg::String>(
        "/nav_mode", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        { this->NavModeCallback(msg); });
    this->joy_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        { this->JoyCallback(msg); });
    this->gazebo_imu_subscriber = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { this->GazeboImuCallback(msg); });
    this->gazebo_model_state_subscriber = ros2_node->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", rclcpp::SystemDefaultsQoS(),
        [this](const gazebo_msgs::msg::ModelStates::SharedPtr msg)
        { this->GazeboModelStatesCallback(msg); });
    this->robot_state_subscriber = ros2_node->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this](const robot_msgs::msg::RobotState::SharedPtr msg)
        { this->RobotStateCallback(msg); });

    // 创建 FSM 状态发布者（供外部导航系统感知当前 FSM 状态）
    this->fsm_state_publisher_ = ros2_node->create_publisher<std_msgs::msg::String>(
        "/fsm_state", rclcpp::SystemDefaultsQoS());

    // service
    this->gazebo_pause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = ros2_node->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = ros2_node->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // RC (航模遥控器) — 仿真中可通过 YAML 开启
    this->InitRC();
    if (this->rc_enabled_)
    {
        this->loop_rc = std::make_shared<LoopFunc>("loop_rc", 0.02, std::bind(&RL_Sim::RCInterface, this));
        this->loop_rc->start();
    }

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos)
    {
        vector = std::vector<float>(this->plot_size, 0);
    }
    for (auto &vector : this->plot_target_joint_pos)
    {
        vector = std::vector<float>(this->plot_size, 0);
    }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    if (this->loop_rc) this->loop_rc->shutdown();
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    this->rc_reader_.stop();
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::StartJointController(const std::string &ros_namespace, const std::vector<std::string> &names)
{
#if defined(USE_ROS1)
    pid_t pid0 = fork();
    if (pid0 == 0)
    {
        std::string cmd = "rosrun controller_manager spawner joint_state_controller ";
        for (const auto &name : names)
        {
            cmd += name + " ";
        }
        cmd += "__ns:=" + ros_namespace;
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
#elif defined(USE_ROS2)
    const char *ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto &name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
#endif
}

void RL_Sim::GetState(RobotState<float> *state)
{
#if defined(USE_ROS1)
    const auto &orientation = this->pose.orientation;
    const auto &angular_velocity = this->vel.angular;
#elif defined(USE_ROS2)
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;
#endif

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        state->motor_state.q[i] = this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.dq[i] = this->joint_velocities[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
        state->motor_state.tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[this->params.Get<std::vector<int>>("joint_mapping")[i]]];
#elif defined(USE_ROS2)
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est;
#endif
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
#if defined(USE_ROS1)
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->joint_publishers_commands[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#elif defined(USE_ROS2)
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau = command->motor_command.tau[i];
#endif
    }

#if defined(USE_ROS1)
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->joint_publishers[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]].publish(this->joint_publishers_commands[i]);
    }
#elif defined(USE_ROS2)
    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
#endif
}

void RL_Sim::RobotControl()
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

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
#if defined(USE_ROS1)
        std_srvs::Empty empty;
        this->gazebo_reset_world_client.call(empty);
#elif defined(USE_ROS2)
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
#endif
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_pause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl
                      << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
#if defined(USE_ROS1)
            std_srvs::Empty empty;
            this->gazebo_unpause_physics_client.call(empty);
#elif defined(USE_ROS2)
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
#endif
            std::cout << std::endl
                      << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

#if defined(USE_ROS1)
void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
}
#elif defined(USE_ROS2)
void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}

void RL_Sim::GazeboModelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
    // 先按 gazebo_model_name 匹配，再 fallback 到 robot_model
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == this->gazebo_model_name || msg->name[i] == "robot_model")
        {
            this->base_lin_vel[0] = static_cast<float>(msg->twist[i].linear.x);
            this->base_lin_vel[1] = static_cast<float>(msg->twist[i].linear.y);
            this->base_lin_vel[2] = static_cast<float>(msg->twist[i].linear.z);
            static bool first = true;
            if (first)
            {
                first = false;
                std::cout << LOGGER::INFO << "lin_vel source: model=" << msg->name[i] << std::endl;
            }
            return;
        }
    }
}
#endif

void RL_Sim::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}

void RL_Sim::NavStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
    this->control.nav_requested_state = msg->data;
    std::cout << LOGGER::INFO << "[Nav] Requested FSM state: " << msg->data << std::endl;
}

void RL_Sim::NavModeCallback(const std_msgs::msg::String::SharedPtr msg)
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

void RL_Sim::JoyCallback(
#if defined(USE_ROS1)
    const sensor_msgs::Joy::ConstPtr &msg
#elif defined(USE_ROS2)
    const sensor_msgs::msg::Joy::SharedPtr msg
#endif
)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

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

    this->control.x = this->joy_msg.axes[1];   // LY
    this->control.y = this->joy_msg.axes[0];   // LX
    this->control.yaw = this->joy_msg.axes[3]; // RX
}

#if defined(USE_ROS1)
void RL_Sim::JointStatesCallback(const robot_msgs::MotorState::ConstPtr &msg, const std::string &joint_controller_name)
{
    this->joint_positions[joint_controller_name] = msg->q;
    this->joint_velocities[joint_controller_name] = msg->dq;
    this->joint_efforts[joint_controller_name] = msg->tau_est;
}
#elif defined(USE_ROS2)
void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}
#endif

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.lin_vel = this->base_lin_vel;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        if (this->control.navigation_mode) {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
        // 用 IMU 加速度积分算线速度，与 Gazebo 真值对比
        {
            static std::vector<float> imu_vel = {0,0,0};
            static auto last_t = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_t).count();
            last_t = now;
            if (dt > 0 && dt < 0.1f) {
                imu_vel[0] += this->gazebo_imu.linear_acceleration.x * dt;
                imu_vel[1] += this->gazebo_imu.linear_acceleration.y * dt;
                imu_vel[2] += (this->gazebo_imu.linear_acceleration.z - 9.81f) * dt;  // 去重力
            }
            static int cnt = 0;
            if (++cnt % 50 == 0) {
                printf("[vel] gt=(%6.3f,%6.3f,%6.3f)  imu=(%6.3f,%6.3f,%6.3f)\n",
                       this->base_lin_vel[0], this->base_lin_vel[1], this->base_lin_vel[2],
                       imu_vel[0], imu_vel[1], imu_vel[2]);
            }
        }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

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

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
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

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
#if defined(USE_ROS1)
        this->plot_real_joint_pos[i].push_back(this->joint_positions[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]]);
        this->plot_target_joint_pos[i].push_back(this->joint_publishers_commands[i].q);
#elif defined(USE_ROS2)
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
#endif
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// ============================================================
// RC 遥控器 (ET08A W.BUS) 接口
// ============================================================
void RL_Sim::InitRC()
{
    this->rc_enabled_ = this->params.Get<bool>("rc_enabled", false);
    if (!this->rc_enabled_)
    {
        std::cout << LOGGER::NOTE << "[RC] RC disabled (rc_enabled=false in YAML). Using keyboard/joy." << std::endl;
        return;
    }

    this->rc_port_ = this->params.Get<std::string>("rc_port", "/dev/ttyUSB0");
    this->rc_baud_ = this->params.Get<int>("rc_baud", 100000);

    std::cout << LOGGER::NOTE << "[RC] Opening " << rc_port_ << " @ " << rc_baud_ << " baud..." << std::endl;

    if (!rc_reader_.open(rc_port_, rc_baud_))
    {
        std::cout << LOGGER::ERROR << "[RC] Failed to open " << rc_port_ << std::endl;
        this->rc_enabled_ = false;
        return;
    }

    rc_reader_.start();
    std::cout << LOGGER::NOTE << "[RC] Serial port opened, background reader started." << std::endl;

    int ch_roll     = this->params.Get<int>("rc_ch_roll",     0);
    int ch_pitch    = this->params.Get<int>("rc_ch_pitch",    1);
    int ch_throttle = this->params.Get<int>("rc_ch_throttle", 2);
    int ch_yaw      = this->params.Get<int>("rc_ch_yaw",      3);
    int ch_sa       = this->params.Get<int>("rc_ch_sa",       4);
    int ch_sb       = this->params.Get<int>("rc_ch_sb",       5);
    int ch_sc       = this->params.Get<int>("rc_ch_sc",       6);
    int ch_rd       = this->params.Get<int>("rc_ch_rd",       7);
    rc_mapper_.setChannelMap(ch_roll, ch_pitch, ch_throttle, ch_yaw,
                             ch_sa, ch_sb, ch_sc, ch_rd);

    std::cout << LOGGER::NOTE << "[RC] Initialized for simulation." << std::endl;
}

void RL_Sim::RCInterface()
{
    static int call_cnt = 0;
    call_cnt++;

    if (!rc_enabled_)
    {
        if (call_cnt == 1)
            std::cout << LOGGER::WARNING << "[RC] rc_enabled_=false, RC interface disabled." << std::endl;
        return;
    }

    if (!rc_reader_.isRunning())
    {
        if (call_cnt == 1)
            std::cout << LOGGER::WARNING << "[RC] Reader not running!" << std::endl;
        return;
    }

    sbus::Frame frame;
    if (!rc_reader_.getLatest(frame))
    {
        if (call_cnt % 50 == 1)
            std::cout << "\r\033[K" << std::flush << "[RC] Waiting for frames... (call #" << call_cnt << ")" << std::flush;
        return;
    }

    Input::Gamepad gp = Input::Gamepad::None;
    float x = 0.0f, y = 0.0f, yaw = 0.0f, rd = 0.0f;

    if (rc_mapper_.process(frame, gp, x, y, yaw, rd))
    {
        if (gp != Input::Gamepad::None)
            this->control.SetGamepad(gp);
        this->control.x   = x;
        this->control.y   = y;
        this->control.yaw = yaw;

        // 诊断输出 (每秒一次)
        static int diag_cnt = 0;
        diag_cnt++;
        if (diag_cnt % 50 == 0)
        {
            std::cout << LOGGER::INFO
                      << "[RC] gp=" << static_cast<int>(gp)
                      << " x=" << std::fixed << std::setprecision(2) << x
                      << " y=" << y << " yaw=" << yaw
                      << " rd=" << rd << std::endl;
        }
    }
}

#if defined(USE_ROS1)
void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}
#endif

int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Sim rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Sim>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
#endif
    return 0;
}
