/**
 * Copyright (c) 2025-2026 Chengjin Pi
 */

#ifndef RL_REAL_MY_DOG_HPP
#define RL_REAL_MY_DOG_HPP

// #define PLOT
#define CSV_LOGGER
#define USE_ROS

#include "rl_sdk.hpp"             // 提供rl基类
#include "observation_buffer.hpp" // 提供历史观测数据缓冲区
#include "inference_runtime.hpp"  // 提供模型推理接口
#include "loop.hpp"               // 提供周期性循环功能
#include "fsm_my_dog.hpp"         // 提供my_dog的状态机定义
#include "sbus_parser.hpp"        // SBUS/W.BUS 协议解析
#include "rc_input_mapper.hpp"    // RC 遥控器到 Gamepad 映射
#include <mutex>
#include "sensor_msgs/msg/joint_state.hpp" // 提供关节状态消息定义

// 条件编译
#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/motor_command.hpp"
#include "robot_msgs/msg/motor_states.hpp"
#endif

#include "matplotlibcpp.h" // 提供绘图功能
namespace plt = matplotlibcpp;

// 定义一些话题名称和常量，这些需要根据my_dog的通信协议和接口定义进行调整
#define TOPIC_ROBOT_CMD "robot_command" // 电机命令话题
#define TOPIC_MOTOR_STATES "motor_states" // 电机状态话题
#define TOPIC_JOINT_STATES "joint_states" // 关节状态话题
#define TOPIC_IMU_STATES "IMU_data" // IMU数据话题


class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv); // 构造函数，主要负责初始化状态机、通信接口、读yaml参数等
    ~RL_Real();                     // 析构函数，主要负责清理通信接口、关闭服务等

#if defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // rl functions
    std::vector<float> Forward() override;                        // 神经网络前向传播函数
    void GetState(RobotState<float> *state) override;             // 提取机器人反馈函数
    void SetCommand(const RobotCommand<float> *command) override; // 发送机器人命令函数
    void RunModel();                                              // 模型推理函数
    void RobotControl();                                          // 机器人控制函数

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard; // 键盘输入循环
    std::shared_ptr<LoopFunc> loop_control;  // 机器人控制循环
    std::shared_ptr<LoopFunc> loop_rl;       // rl循环
    std::shared_ptr<LoopFunc> loop_rc;       // RC遥控器输入循环
    std::shared_ptr<LoopFunc> loop_plot;     // 绘图循环

    // plot
    const int plot_size = 100;                                                  // 绘图数据点数量
    std::vector<int> plot_t;                                                    // 绘图时间轴
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos; // 绘图数据
    void Plot();                                                                // 绘图函数

    // my_dog interface
    void InitLowCmd(); // 初始化发出的底层命令
    void MotorStatesCallback(const robot_msgs::msg::MotorStates::SharedPtr msg); // 电机状态回调函数
    void IMUStateCallback(const sensor_msgs::msg::Imu::SharedPtr msg); // IMU状态回调函数
    robot_msgs::msg::MotorStates my_dog_motor_states; // 电机状态消息，用于通信
    robot_msgs::msg::RobotCommand my_dog_robot_command; // 机器人命令消息，用于通信
    robot_msgs::msg::MotorCommand my_dog_motor_command; // 电机命令消息，用于通信
    sensor_msgs::msg::Imu my_dog_imu_state;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr my_dog_robot_command_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_velocity_publisher_;
    rclcpp::Subscription<robot_msgs::msg::MotorStates>::SharedPtr my_dog_motor_states_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr my_dog_imu_state_subscriber;
    
    // mutex
    std::mutex imu_state_mutex_; // IMU状态互斥锁，用于保护IMU状态的并发访问
    std::mutex motor_states_mutex_; // 电机状态互斥锁，用于保护电机状态的并发访问
    std::mutex command_mutex_; // 命令互斥锁，用于保护命令的并发访问
    std::mutex robot_state_mutex_; // 机器人状态互斥锁，用于保护机器人状态的并发访问(this->robot_state)

    // RC (航模遥控器)
    sbus::Reader  rc_reader_;
    rc::Mapper    rc_mapper_;
    bool          rc_enabled_ = false;
    std::string   rc_port_ = "/dev/ttyUSB0";
    int           rc_baud_ = 100000;
    float         rc_rd_   = 0.0f;       // RD 旋钮 [0, 1]
    bool          rc_poweroff_triggered_ = false;  // RD 关机防重复
    int           rc_last_retry_ = 0;             // 上次重试的 motiontime
    void RCInterface();       // RC 遥控器输入处理
    void InitRC();            // 初始化 RC 串口和映射

    // others
    std::vector<float> mapped_joint_positions; // 映射后的关节位置，具体映射关系需要根据my_dog的关节定义进行调整
    std::vector<float> mapped_joint_velocites; // 映射后的关节速度，具体映射关系需要根据my_dog的关节定义进行调整

 #if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;                                    // 从cmd_vel话题接收的速度命令
    ros::Subscriber cmd_vel_subscriber;                              // cmd_vel话题订阅者
    ros::Subscriber joy_subscriber;                                  // joy话题订阅者
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msgs); // cmd_vel话题回调函数，处理接收到的速度命令
    void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);         // joy话题回调函数，处理手柄输入
#elif defined(USE_ROS2)
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Twist body_velocity_msg_;
    sensor_msgs::msg::Joy joy_msg;
    bool lt_was_pressed = false;  // LT 上升沿检测
    std::vector<std::string> joint_names_;
    sensor_msgs::msg::JointState joint_states_msg;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_state_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_mode_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_state_publisher_;
    std::string last_fsm_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void NavStateCallback(const std_msgs::msg::String::SharedPtr msg);
    void NavModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void PublishJointStates(); // 发布关节状态函数
#endif
};

#endif // RL_REAL_MY_DOG_HPP