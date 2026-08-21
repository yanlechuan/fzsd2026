#include "motor_ros2/can_thread_manager.h"
#include <iostream>
#include <chrono>

CanDeviceThread::CanDeviceThread(const std::string& interface) : can_interface_(interface){}

CanDeviceThread::~CanDeviceThread()
{
    stop();
}

void CanDeviceThread::addMotor(std::unique_ptr<RobStrideMotor> motor)
{
    motors_on_device_.push_back(std::move(motor));
    latest_states_.resize(motors_on_device_.size());
}

void CanDeviceThread::start()
{
    running_ = true;
    thread_handle_ = std::thread(&CanDeviceThread::threadFunction, this);
}

void CanDeviceThread::disableAllMotors()
{
    // 为所有电机发送disable命令
    for(auto& motor : motors_on_device_)
    {
        motor->Disenable_Motor(0);
        std::this_thread::sleep_for(std::chrono::microseconds(100)); // 给一点时间让命令生效
    }
}

void CanDeviceThread::stop()
{
    running_ = false;
    command_cv_.notify_one(); // 唤醒线程处理命令

    if(thread_handle_.joinable())
    {
        thread_handle_.join();
    }

}

void CanDeviceThread::sendCommand(const Motor_CommandItem& cmd)
{
    {
        std::lock_guard<std::mutex> lock(command_queue_mutex_);
        command_queue_.push(cmd);
    }
    command_cv_.notify_one(); // 唤醒线程处理命令
}

std::vector<robot_msgs::msg::MotorState> CanDeviceThread::getLatestStates()
{
    std::lock_guard<std::mutex> lock(states_mutex_);
    return latest_states_;
}

void CanDeviceThread::threadFunction()
{
    int sum = 0;
    // 初始化电机
    for(auto& motor : motors_on_device_)
    {
        motor->Get_RobStrite_Motor_parameter(0x7005);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 增加延时以确保参数读取完成
        // 多次发送enable命令以确保电机接收
        for(int i = 0; i < 5; ++i) {
            motor->enable_motor();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 短暂延时
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 确保电机完全使能
        sum++;
        std::cout<<"sum = "<<sum<<std::endl;
    }


    // 添加频率监测变量
    int cycle_count = 0;
    auto frequency_monitor_start = std::chrono::steady_clock::now();
    
    while(running_)
    {
        auto loop_start_time = std::chrono::steady_clock::now();
        
        // 处理命令队列
        std::queue<Motor_CommandItem> local_queue;
        {
            std::unique_lock<std::mutex> lock(command_queue_mutex_);
            command_cv_.wait_for(lock,std::chrono::microseconds(100),[this]{return !command_queue_.empty() || !running_;});

            // 将队列中的任务转移到本地处理
            while(!command_queue_.empty())
            {
                local_queue.push(command_queue_.front());
                command_queue_.pop();
            }
        }

        // 串行处理该can设备上所有的电机命令
        std::vector<bool> processed(motors_on_device_.size(), false);

        while(!local_queue.empty())
        {
            auto cmd = local_queue.front();
            local_queue.pop();
            // 找到对应的电机进行控制
            for (size_t i = 0; i < motors_on_device_.size(); ++i)
            {
                if(motors_on_device_[i]->motor_id == cmd.motor_id && !processed[i])
                {
                    // 使用正确的电机控制命令
                    auto [pos,vel,tq,temp] = motors_on_device_[i]->send_motion_command(
                        cmd.torque, cmd.position, cmd.velocity, cmd.kp, cmd.kd);
                

                    // 更新电机状态
                    {
                        std::lock_guard<std::mutex> lock(states_mutex_);
                        latest_states_[i].q = pos;
                        latest_states_[i].dq = vel;
                        latest_states_[i].ddq = 0.0f;
                        latest_states_[i].tau_est = tq;
                        latest_states_[i].cur = temp;
                    }

                    processed[i] = true;

                    break;
                }   
            }
        }
        
        // 对未处理的电机进行状态轮询（在多线程调度中可能出现电机指令丢失的情况）
        for (size_t i = 0; i < motors_on_device_.size(); ++i)
        {
            if (!processed[i])
            {
                // 发送空命令来获取状态（如果需要）
                try {
                    auto [pos, vel, tq, temp] = motors_on_device_[i]->send_motion_command(
                        0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
                    {
                        std::lock_guard<std::mutex> lock(states_mutex_);
                        latest_states_[i].q = pos;
                        latest_states_[i].dq = vel;
                        latest_states_[i].ddq = 0.0f;
                        latest_states_[i].tau_est = tq;
                        latest_states_[i].cur = temp;
                    }
                } catch (...) {
                    // 忽略错误，保持之前的状态
                }
            }
        }

        // 动态调整延时，基于处理时间计算合适的间隔
        auto processing_end = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(processing_end - loop_start_time).count();
        
        // 设定期望的控制周期 (例如 2ms = 500Hz)
        const int desired_period_us = 2000; // 2ms = 2000微秒
        
        if(processing_time < desired_period_us) {
            std::this_thread::sleep_for(std::chrono::microseconds(desired_period_us - processing_time));
        }
        // 如果处理时间超过期望周期，则立即进入下一次循环
        
        // 频率监测：每秒输出一次频率信息
        cycle_count++;
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - frequency_monitor_start).count();
        
        if (duration >= 1000) {  // 每秒输出一次
            double actual_frequency = (cycle_count * 1000.0) / duration;
            std::cout << "[DEBUG] CAN Interface " << can_interface_ << " Control Frequency: " << actual_frequency << " Hz (" << cycle_count << " cycles in " << duration << " ms)" << std::endl;
            std::cout << "        Processed Motors: " << std::count(processed.begin(), processed.end(), true) << "/" << motors_on_device_.size() << std::endl;
            std::cout << "        Processing Time: " << processing_time << " μs" << std::endl;
            
            cycle_count = 0;
            frequency_monitor_start = current_time;
        }
    }
}