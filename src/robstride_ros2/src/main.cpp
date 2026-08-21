#include "motor_ros2/motor_cfg.h"
#include "stdint.h"
#include <atomic>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/motor_states.hpp"
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unistd.h>
#include <vector>
#include <chrono>
#include "motor_ros2/can_thread_manager.h"

class MotorControlSample : public rclcpp::Node
{
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node")
        // 初始化 motors 列表（根据实际电机数量与 id 修改）
  {
    // 参数化配置（可通过 YAML 文件或命令行参数覆盖）
    this->declare_parameter<std::vector<std::string>>("can_ifaces", std::vector<std::string>{"can0", "can1", "can2", "can3", "can4"});
    this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
    // motor_iface_map 指定每个 motor 对应的 can_ifaces 索引（长度应与 motor_ids 相同）
    this->declare_parameter<std::vector<int64_t>>("motor_iface_map", std::vector<int64_t>{0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3});
    this->declare_parameter<int>("master_id", 255);
    this->declare_parameter<int>("actuator_type", 2);
    // 可选：为每个电机分别指定 master_id 与 actuator_type（与 motor_ids 对应长度）
    this->declare_parameter<std::vector<int64_t>>("motor_master_ids", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("motor_actuator_types", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("motor_can_ids", std::vector<int64_t>{0x01,0x02,0x03,0x01,0x02,0x03,0x01,0x02,0x03,0x01,0x02,0x03});
    this->declare_parameter<std::string>("motor_states_topic", "motor_states");
    this->declare_parameter<std::string>("robot_command_topic", "robot_command");
    this->declare_parameter<int>("publish_rate", 400);

    std::vector<std::string> can_ifaces;
    this->get_parameter("can_ifaces", can_ifaces);
    std::vector<int64_t> motor_ids;
    this->get_parameter("motor_ids", motor_ids);
    std::vector<int64_t> motor_iface_map;
    this->get_parameter("motor_iface_map", motor_iface_map);
    int master_id = this->get_parameter("master_id").as_int();
    int actuator_type = this->get_parameter("actuator_type").as_int();
    std::vector<int64_t> motor_master_ids;
    this->get_parameter("motor_master_ids", motor_master_ids);
    std::vector<int64_t> motor_actuator_types;
    this->get_parameter("motor_actuator_types", motor_actuator_types);
    std::vector<int64_t> motor_can_ids;
    this->get_parameter("motor_can_ids", motor_can_ids);
    std::string motor_states_topic = this->get_parameter("motor_states_topic").as_string();
    std::string robot_command_topic = this->get_parameter("robot_command_topic").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_int();

    // 为每一个CAN接口创建一个线程管理器
    for(const auto& iface : can_ifaces)
    {
        can_threads_[iface] = std::make_unique<CanDeviceThread>(iface);
    }

    // 根据参数创建 motor 实例，支持每个电机绑定到不同 can_iface
    for (size_t i = 0; i < motor_ids.size(); ++i)
    {
      int64_t id = motor_ids[i];
      int iface_idx = 0;
      if (i < motor_iface_map.size())
        iface_idx = static_cast<int>(motor_iface_map[i]);
      if (iface_idx < 0 || iface_idx >= static_cast<int>(can_ifaces.size()))
        iface_idx = 0;
      const std::string &iface = can_ifaces[iface_idx];
      // per-motor master_id / actuator_type fallback
      int this_master = master_id;
      if (i < motor_master_ids.size())
        this_master = static_cast<int>(motor_master_ids[i]);
      int this_actuator = actuator_type;
      if (i < motor_actuator_types.size())
        this_actuator = static_cast<int>(motor_actuator_types[i]);
      uint8_t this_can_eid = static_cast<uint8_t>(id);
      if (i < motor_can_ids.size())
        this_can_eid = static_cast<uint8_t>(motor_can_ids[i]);
      
      // 创建电机实例并添加到对应接口的线程中
      auto motor = std::make_unique<RobStrideMotor>(iface, this_master, id, this_actuator, this_can_eid);
      can_threads_[iface]->addMotor(std::move(motor));
    }

    // 带横杠的是成员变量，不带的是局部变量
    this->can_ifaces_ = can_ifaces;
    this->motor_ids_ = motor_ids;
    this->motor_iface_map_.resize(motor_ids.size());
    for(size_t i = 0; i < motor_ids.size(); ++i )
    {
      this->motor_iface_map_[i] = static_cast<int>(motor_iface_map[i]);
    }

    // 启动所有CAN接口的线程
    for (auto& pair : can_threads_)
    {
      pair.second->start();
    }

    // 订阅/发布（主题名可通过参数配置）
    state_pub_ = this->create_publisher<robot_msgs::msg::MotorStates>(motor_states_topic, 10);
    cmd_sub_ = this->create_subscription<robot_msgs::msg::RobotCommand>(
        robot_command_topic, 10,
        [this](const robot_msgs::msg::RobotCommand::SharedPtr msg)
        {
          this->handle_robot_command(msg);
        });

    // 启动状态发布线程
    state_publish_thread_ = std::thread(&MotorControlSample::state_publish_loop, this);
  }


  ~MotorControlSample()
  {
    running_ = false;

    // 先向所有电机发送disable命令
    for (auto& pair : can_threads_)
    {
      pair.second->disableAllMotors();
    }

    // 停止所有CAN设备线程
    for (auto& pair : can_threads_)
    {
      pair.second->stop();
    }

    if(state_publish_thread_.joinable())
      state_publish_thread_.join();
  }


  void handle_robot_command(const robot_msgs::msg::RobotCommand::SharedPtr msg)
  {
    // 将命令分发到相应的CAN线程
    for(size_t i = 0; i < msg->motor_command.size(); ++i)
    {
      const auto &cmd = msg->motor_command[i];
      int64_t motor_id = motor_ids_[i];
      int iface_idx = motor_iface_map_[i];
      const std::string& iface = can_ifaces_[iface_idx];

      Motor_CommandItem cmd_item;
      cmd_item.torque = cmd.tau;
      cmd_item.position = cmd.q;
      cmd_item.velocity = cmd.dq;
      cmd_item.kp = cmd.kp;
      cmd_item.kd = cmd.kd;
      cmd_item.motor_id = static_cast<uint8_t>(motor_id);

      can_threads_[iface]->sendCommand(cmd_item);
    }
  }

private:
  void state_publish_loop()
  {
    auto last_publish = std::chrono::steady_clock::now();
    int target_period_ms = 1000 / publish_rate_;
    
    // 频率监测变量
    int publish_cycle_count = 0;
    auto publish_frequency_monitor_start = std::chrono::steady_clock::now();
    
    while(running_)
    {
      auto start_time = std::chrono::steady_clock::now();

      // 收集所有CAN设备的状态
      robot_msgs::msg::MotorStates state_msg;
      for(const auto& iface : can_ifaces_)
      {
        // 获取状态副本以避免并发访问问题
        auto states_copy = can_threads_[iface]->getLatestStates();
        state_msg.motor_state.insert(state_msg.motor_state.end(), 
                                    states_copy.begin(), 
                                    states_copy.end());
      }

      state_pub_->publish(state_msg);

      // 控制发布频率
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();

      if(elapsed < target_period_ms)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(target_period_ms - elapsed));
      }
      else
      {
        // 如果处理时间超过目标周期，立即进行下一次循环
        RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, 
                            "State publishing took longer than target period (%d ms)", target_period_ms);
      }
      
      // 发布频率监测：每秒输出一次频率信息
      publish_cycle_count++;
      auto current_time = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - publish_frequency_monitor_start).count();
      
      if (duration >= 1000) {  // 每秒输出一次
        double actual_publish_frequency = (publish_cycle_count * 1000.0) / duration;
        std::cout << "[DEBUG] State Publish Frequency: " << actual_publish_frequency << " Hz (" << publish_cycle_count << " publishes in " << duration << " ms)" << std::endl;
        std::cout << "        Target Rate: " << publish_rate_ << " Hz" << std::endl;
        std::cout << "        Total Motors: " << state_msg.motor_state.size() << std::endl;
        
        publish_cycle_count = 0;
        publish_frequency_monitor_start = current_time;
      }
    }
  }
  

private:
  std::map<std::string, std::unique_ptr<CanDeviceThread>> can_threads_;
  std::thread state_publish_thread_;
  std::atomic<bool> running_ = true;

  std::vector<std::string> can_ifaces_;
  std::vector<int64_t> motor_ids_;
  std::vector<int> motor_iface_map_;

  rclcpp::Publisher<robot_msgs::msg::MotorStates>::SharedPtr state_pub_;
  rclcpp::Subscription<robot_msgs::msg::RobotCommand>::SharedPtr cmd_sub_;

  int publish_rate_ = 100;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}