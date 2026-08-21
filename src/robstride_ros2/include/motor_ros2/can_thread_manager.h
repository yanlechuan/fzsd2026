#include <thread>
#include <vector>
#include <mutex>    
#include <queue>
#include <condition_variable>
#include <functional>
#include <atomic>
#include "robot_msgs/msg/motor_state.hpp"
#include "motor_ros2/motor_cfg.h"


struct Motor_CommandItem
{
    float torque;
    float position;
    float velocity;
    float kp;
    float kd;
    uint8_t motor_id;
};

class CanDeviceThread
{ 
private:
    std::string can_interface_;   
    std::vector<std::unique_ptr<RobStrideMotor>> motors_on_device_;
    std::thread thread_handle_;
    std::atomic<bool> running_{false};
    std::mutex command_queue_mutex_;
    std::mutex states_mutex_;
    std::queue<Motor_CommandItem> command_queue_;
    std::condition_variable command_cv_;

    // 用于发布电机状态
    std::vector<robot_msgs::msg::MotorState> latest_states_;

public:
    CanDeviceThread(const std::string& can_interface);
    ~CanDeviceThread();
    
    void start();
    void stop();
    void disableAllMotors();
    void addMotor(std::unique_ptr<RobStrideMotor> motor);   

    void sendCommand(const Motor_CommandItem& cmd);
    std::vector<robot_msgs::msg::MotorState> getLatestStates();
private:
    void threadFunction();
    void updateState(size_t index, float pos, float vel, float tq, float temp);
};