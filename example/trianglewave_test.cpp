#include <unistd.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <cstdlib>  // 添加此行用于system函数
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <atomic>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "torque_sensor.h"
#include <iomanip>
#include <sstream>
#include <algorithm>

// 电机类型定义
#define MOTOR_TYPE MotorType::B1  // 可以修改为 MotorType::A1 或其他类型

// 控制参数定义
#define WORK_KP 5.0f
#define WORK_KD 30.0f
#define VELOCITY_KP 0.0f    // 速度控制模式下位置环增益为0
#define VELOCITY_KD 50.0f   // 速度控制模式下的速度环增益
#define CONTROL_PERIOD_US 10  // 控制周期：10微秒 (100kHz)
#define MAX_RETRY_COUNT 3     // 最大重试次数
#define RETRY_DELAY_US 5      // 重试延时（微秒）
#define DEFAULT_CURRENT 1.0f  // 默认安全电流值（A）

// 控制模式枚举
enum class ControlMode {
    POSITION,
    VELOCITY
};

// 生成三角波速度的函数
float generateTriangleWaveVelocity(float time, float amplitude, float frequency) {
    // 计算周期
    float period = 1.0f / frequency;
    // 计算当前时间在周期内的位置
    float t = fmod(time, period);
    
    // 计算速度（三角波的导数是方波）
    if (t < period / 2.0f) {
        // 上升段：正向速度
        return 2.0f * amplitude * frequency;
    } else {
        // 下降段：负向速度
        return -2.0f * amplitude * frequency;
    }
}

// 生成三角波位置轨迹的函数
float generateTriangleWavePosition(float time, float amplitude, float frequency) {
    // 计算周期
    float period = 1.0f / frequency;
    // 计算当前时间在周期内的位置
    float t = fmod(time, period);
    // 计算上升和下降的斜率
    float slope = 2.0f * amplitude / period;
    
    // 生成三角波
    if (t < period / 2.0f) {
        // 上升段：从0上升到amplitude
        return slope * t;
    } else {
        // 下降段：从amplitude下降到0
        return amplitude - slope * (t - period / 2.0f);
    }
}

// 速度控制模式下的位置计算函数
float calculatePosition(float time, float velocity_rpm, float target_angle_deg) {
    // 将RPM转换为rad/s
    float velocity_rad_per_sec = velocity_rpm * (2.0f * M_PI / 60.0f);
    
    // 计算一个完整周期的时间
    float target_angle_rad = target_angle_deg * (M_PI / 180.0f);
    float period = 2.0f * target_angle_rad / velocity_rad_per_sec;
    
    // 计算当前时间在周期内的位置
    float t = fmod(time, period);
    
    // 计算当前位置
    if (t < period / 2.0f) {
        // 上升段
        return velocity_rad_per_sec * t;
    } else {
        // 下降段
        return target_angle_rad - velocity_rad_per_sec * (t - period / 2.0f);
    }
}

// 保存数据到文件的函数
void saveDataToFile(std::ofstream& file, float time, float desired_torque, float actual_torque, 
                   float velocity, float position, float desired_position, float power, double sensor_torque) {
    file << time << "," 
         << desired_torque << "," 
         << actual_torque << "," 
         << velocity << "," 
         << position << "," 
         << desired_position << "," 
         << power << ","
         << sensor_torque << "\n";
}

// 获取用户输入，如果用户直接按回车则使用默认值
float getInputWithDefault(const std::string& prompt, float default_value) {
    std::string input;
    std::cout << prompt << " [" << default_value << "]: ";
    std::getline(std::cin, input);
    
    if (input.empty()) {
        return default_value;
    }
    
    try {
        return std::stof(input);
    } catch (...) {
        std::cout << "Invalid input, using default value: " << default_value << std::endl;
        return default_value;
    }
}

// 设置当前位置为零位的函数
void setCurrentPositionAsZero(SerialPort& serial, MotorCmd& cmd, MotorData& data, float gear_ratio, float& zero_position) {
    std::cout << "\nSetting current position as zero position..." << std::endl;
    
    // 1. 首先将电机设置为零刚度模式
    cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode::FOC);
    cmd.kp = 0.0f;  // 设置位置环增益为0
    cmd.kd = 0.0f;  // 设置速度环增益为0
    cmd.tau = 0.0f; // 设置力矩为0
    
    // 等待电机稳定
    for(int i = 0; i < 100; i++) {  // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during zero position setting!" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));  // 2000微秒延时
    }
    
    // 2. 记录当前位置作为新的零位（考虑减速比）
    zero_position = data.q;  // 记录转子位置作为零位
    float output_position = zero_position / gear_ratio;  // 转换为输出轴位置
    std::cout << "Current rotor position: " << zero_position << " rad" << std::endl;
    std::cout << "Current output position: " << output_position << " rad ("
              << output_position * (180.0f / M_PI) << " degrees)" << std::endl;
    
    cmd.q = zero_position;  // 设置当前位置为零位

    // 3. 恢复正常控制参数
    cmd.kp = WORK_KP;  // 恢复位置环增益
    cmd.kd = WORK_KD; // 恢复速度环增益
    
    // 4. 等待电机稳定在新的控制参数下
    for(int i = 0; i < 100; i++) {  // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during control parameter restoration!" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));  // 2000微秒延时
    }
    
    std::cout << "Zero position setting completed" << std::endl;
}

// 修改绘图函数
void plotData(const std::string& filename) {
    // 使用conda的Python
    std::string command = "cd ../example && /home/wenbo/anaconda3/envs/torque-bench/bin/python plot_data.py data/" + filename;
    std::cout << "\nGenerating and saving plots..." << std::endl;
    int result = system(command.c_str());
    if (result != 0) {
        std::cerr << "Error: Failed to generate and save plots!" << std::endl;
    } else {
        std::cout << "Plots have been saved to figure/" << filename << ".png" << std::endl;
    }
}

// 创建目录的函数
void ensureDataDirectory() {
    mkdir("data", 0777);
    mkdir("figure", 0777);  // 创建figure目录
}

// 共享数据结构
struct SharedData {
    std::mutex mutex;
    double sensor_torque;
    std::chrono::time_point<std::chrono::high_resolution_clock> sensor_timestamp;
    bool has_new_data;

    SharedData() : sensor_torque(0.0), has_new_data(false) {}
};

// 扭矩传感器读取线程函数
void torqueSensorThread(std::atomic<bool>& running, SharedData& shared_data, TorqueSensor& sensor) {
    while (running) {
        double torque = sensor.readTorque();
        auto now = std::chrono::high_resolution_clock::now();

        {
            std::lock_guard<std::mutex> lock(shared_data.mutex);
            shared_data.sensor_torque = torque;
            shared_data.sensor_timestamp = now;
            shared_data.has_new_data = true;
        }

        // 控制读取频率，与电机控制频率相匹配
        std::this_thread::sleep_for(std::chrono::microseconds(CONTROL_PERIOD_US));
    }
}

// 修改打印测试参数函数
void printTestParameters(ControlMode mode, float amplitude_or_velocity, float frequency_or_target, 
                       float cycles, float current, const std::string& filename, float run_time) {
    std::cout << "\n========== Test Parameters ==========\n"
              << "Test Type: Triangle Wave " << (mode == ControlMode::POSITION ? "Position" : "Velocity") << " Control\n";
    
    if (mode == ControlMode::POSITION) {
        std::cout << "Amplitude: " << amplitude_or_velocity << " degrees\n"
                  << "Frequency: " << frequency_or_target << " Hz\n";
    } else {
        std::cout << "Velocity: " << amplitude_or_velocity << " RPM = " << std::fixed << std::setprecision(2) << amplitude_or_velocity * (2.0f * M_PI / 60.0f) << " rad/s\n"
                  << "Target Angle: " << frequency_or_target << " degrees\n";
    }
    
    std::cout << "Cycles: " << cycles << "\n"
              << "Control Current: " << current << " A\n"
              << "Run Time: " << run_time << " s\n"
              << "Data File: " << filename << ".csv\n"
              << "===================================\n" << std::endl;
}

// 修改生成自动文件名函数
std::string generateFileName(ControlMode mode, float amplitude_or_velocity, float frequency_or_target, 
                           float cycles, float current) {
    std::stringstream ss;
    ss << "triangle" << (mode == ControlMode::POSITION ? "Position" : "Velocity");
    std::string str = ss.str();
    
    if (mode == ControlMode::POSITION) {
        ss.str("");
        ss << str << "_amp" << std::fixed << std::setprecision(2) << amplitude_or_velocity;
    } else {
        ss.str("");
        ss << str << "_vel" << std::fixed << std::setprecision(2) << amplitude_or_velocity << "rpm";
    }
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    if (mode == ControlMode::POSITION) {
        ss.str("");
        ss << str << "_freq" << frequency_or_target;
    } else {
        ss.str("");
        ss << str << "_target" << frequency_or_target << "deg";
    }
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_cyc" << cycles;
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_cur" << current << "A";
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    return str;
}

int main() {
    // 确保data目录存在
    ensureDataDirectory();

    // 初始化串口
    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;

    // 初始化扭矩传感器
    TorqueSensor torqueSensor;
    bool sensor_connected = torqueSensor.initialize("/dev/ttyUSB1", 115200);
    if (!sensor_connected) {
        std::cerr << "\033[33m警告: 扭矩传感器连接失败! 程序将继续执行，但不会记录传感器数据。\033[0m" << std::endl;
    }

    // 设置电机类型为B1
    cmd.motorType = MOTOR_TYPE;
    data.motorType = MOTOR_TYPE;

    // 获取减速比
    float gear_ratio = queryGearRatio(MOTOR_TYPE);
    std::cout << "Gear ratio: " << gear_ratio << std::endl;

    // 设置电机控制参数（力矩控制模式）
    cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = 0.0f;   // 初始位置环增益为0
    cmd.kd = 0.0f;   // 初始速度环增益为0
    cmd.q = 0.0f;    // 位置
    cmd.dq = 0.0f;   // 速度
    cmd.tau = 0.0f;  // 力矩

    // 测试电机通信
    std::cout << "Testing motor communication..." << std::endl;
    if (!serial.sendRecv(&cmd, &data)) {
        std::cerr << "Error: Failed to communicate with motor!" << std::endl;
        return 1;
    }
    std::cout << "Motor communication successful!" << std::endl;

    // 清除输入缓冲区
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // 在开始三角波运动之前，设置当前位置为零位
    float zero_position = 0.0f;  // 用于存储零位位置
    setCurrentPositionAsZero(serial, cmd, data, gear_ratio, zero_position);

    // 获取用户输入参数（带默认值）
    std::string mode_input;
    ControlMode control_mode;
    std::cout << "Select control mode:\n"
              << "1. Position Control\n"
              << "2. Velocity Control\n"
              << "Enter choice [2]: ";  // 修改默认值为2
    std::getline(std::cin, mode_input);
    control_mode = (mode_input == "1") ? ControlMode::POSITION : ControlMode::VELOCITY;  // 修改判断条件

    float amplitude_or_velocity;
    float frequency_or_target;
    if (control_mode == ControlMode::POSITION) {
        amplitude_or_velocity = getInputWithDefault("Enter amplitude (degrees)", 20.0f);
        float amplitude_rad = amplitude_or_velocity * (M_PI / 180.0f);
        std::cout << "Amplitude: " << amplitude_or_velocity << " degrees = " << std::fixed << std::setprecision(3) << amplitude_rad << " rad" << std::endl;
        frequency_or_target = getInputWithDefault("Enter frequency (Hz)", 1.0f);
    } else {
        amplitude_or_velocity = getInputWithDefault("Enter velocity (RPM)", 30.0f);
        float velocity_rad_per_sec = amplitude_or_velocity * (2.0f * M_PI / 60.0f);
        std::cout << "Velocity: " << amplitude_or_velocity << " RPM = " << std::fixed << std::setprecision(2) << velocity_rad_per_sec << " rad/s" << std::endl;
        frequency_or_target = getInputWithDefault("Enter target angle (degrees)", 20.0f);
        float target_angle_rad = frequency_or_target * (M_PI / 180.0f);
        std::cout << "Target Angle: " << frequency_or_target << " degrees = " << std::fixed << std::setprecision(3) << target_angle_rad << " rad" << std::endl;
    }
    
    float cycles = getInputWithDefault("Enter number of cycles", 3.0f);
    float current = getInputWithDefault("Enter control current (A)", DEFAULT_CURRENT);
    
    // 在速度控制模式下，计算并显示预计运行时间
    if (control_mode == ControlMode::VELOCITY) {
        float velocity_rad_per_sec = amplitude_or_velocity * (2.0f * M_PI / 60.0f);  // RPM to rad/s
        float target_angle_rad = frequency_or_target * (M_PI / 180.0f);  // degrees to rad
        float time_to_target = target_angle_rad / velocity_rad_per_sec;  // 到达目标角度的时间
        float time_per_cycle = 2.0f * time_to_target;  // 来回一次的时间
        float estimated_run_time = time_per_cycle * cycles;
        std::cout << "Estimated run time: " << std::fixed << std::setprecision(2) << estimated_run_time << " seconds" << std::endl;
        std::cout << "  - Time to target: " << std::fixed << std::setprecision(2) << time_to_target << " seconds" << std::endl;
        std::cout << "  - Time per cycle: " << std::fixed << std::setprecision(2) << time_per_cycle << " seconds" << std::endl;
        std::cout << "  - Total cycles: " << cycles << std::endl;
        std::cout << "  - Velocity: " << std::fixed << std::setprecision(2) << velocity_rad_per_sec << " rad/s" << std::endl;
        std::cout << "  - Target angle: " << std::fixed << std::setprecision(2) << target_angle_rad << " rad" << std::endl;
    }
    
    // 修改文件保存路径
    std::string filename;
    std::string auto_filename = generateFileName(control_mode, amplitude_or_velocity, 
                                               frequency_or_target, cycles, current);
    std::cout << "Enter filename [" << auto_filename << "]: ";
    std::getline(std::cin, filename);
    if (filename.empty()) {
        filename = auto_filename;
    }
    
    // 构建完整的文件路径（相对于example目录）
    std::string full_path = "../example/data/" + filename + ".csv";
    
    // 计算运行时间
    float run_time;
    if (control_mode == ControlMode::POSITION) {
        run_time = cycles / frequency_or_target;
    } else {
        // 在速度控制模式下，根据速度和目标角度计算运行时间
        float target_angle_rad = frequency_or_target * (M_PI / 180.0f);
        float time_per_cycle = 2.0f * target_angle_rad / (amplitude_or_velocity * (2.0f * M_PI / 60.0f));  // 来回一次的时间
        run_time = time_per_cycle * cycles;
    }
    
    // 打印测试参数
    printTestParameters(control_mode, amplitude_or_velocity, frequency_or_target, 
                       cycles, current, filename, run_time);
    
    std::cout << "\nRunning for " << run_time << " seconds" << std::endl;

    // 创建数据文件
    std::ofstream data_file(full_path);
    if (!data_file.is_open()) {
        std::cerr << "Error: Could not open file " << full_path << std::endl;
        return 1;
    }
    data_file << "Time(s),d_Torque(Nm),a_Torque(Nm),Velocity(rad/s),Position(rad),Desired_Position(rad),Power(W),Sensor_Torque(Nm)\n";

    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    float elapsed_time = 0.0f;

    std::cout << "Starting motor control..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // 主控制循环
    float last_successful_time = 0.0f;
    float last_successful_position = 0.0f;
    bool need_resend = false;
    auto last_control_time = std::chrono::high_resolution_clock::now();
    int retry_count = 0;  // 当前重试次数
    
    // 初始化共享数据和线程控制
    SharedData shared_data;
    std::atomic<bool> running(true);
    double last_valid_sensor_torque = 0.0;  // 保存最后一次有效的传感器数据

    // 只有在传感器连接成功时才启动传感器线程
    std::thread sensor_thread;
    if (sensor_connected) {
        sensor_thread = std::thread(torqueSensorThread, std::ref(running), std::ref(shared_data), std::ref(torqueSensor));
    }

    while (elapsed_time < run_time) {
        // 计算时间间隔
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last_control = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - last_control_time
        ).count();
        
        // 如果距离上次控制时间不足一个控制周期，则等待
        if (time_since_last_control < CONTROL_PERIOD_US) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(CONTROL_PERIOD_US - time_since_last_control)
            );
            continue;
        }
        
        float desired_angle_deg;
        float desired_angle_rad;
        float rotor_angle;
        float desired_velocity;
        
        if (control_mode == ControlMode::POSITION) {
            if (!need_resend) {
                // 位置控制模式
                desired_angle_deg = generateTriangleWavePosition(elapsed_time, amplitude_or_velocity, frequency_or_target);
                desired_angle_rad = desired_angle_deg * (M_PI / 180.0f);
                rotor_angle = (desired_angle_rad * gear_ratio) + zero_position;
                desired_velocity = 0.0f;  // 位置控制模式下不直接控制速度
                
                // 设置位置控制参数
                cmd.kp = WORK_KP;
                cmd.kd = WORK_KD;
                retry_count = 0;  // 重置重试计数
            } else {
                // 使用上一次成功的位置
                desired_angle_deg = last_successful_position;
                desired_angle_rad = desired_angle_deg * (M_PI / 180.0f);
                rotor_angle = (desired_angle_rad * gear_ratio) + zero_position;
                desired_velocity = 0.0f;
            }
        } else {
            // 速度控制模式
            // 计算一个完整周期的时间
            float velocity_rad_per_sec = amplitude_or_velocity * (2.0f * M_PI / 60.0f);  // RPM to rad/s
            float target_angle_rad = frequency_or_target * (M_PI / 180.0f);  // degrees to rad
            float time_per_cycle = 2.0f * target_angle_rad / velocity_rad_per_sec;  // 来回一次的时间
            
            // 计算当前时间在周期内的位置
            float t = fmod(elapsed_time, time_per_cycle);
            
            // 根据时间决定速度方向
            if (t < time_per_cycle / 2.0f) {
                // 前半周期：正向速度
                desired_velocity = velocity_rad_per_sec;
            } else {
                // 后半周期：负向速度
                desired_velocity = -velocity_rad_per_sec;
            }
            
            // 设置速度控制参数
            cmd.kp = VELOCITY_KP;  // 位置环增益为0
            cmd.kd = VELOCITY_KD;  // 速度环增益
        }

        // 更新电机命令
        if (control_mode == ControlMode::POSITION) {
            cmd.q = rotor_angle;
            cmd.dq = desired_velocity * gear_ratio;  // 速度需要乘以减速比，因为cmd.dq是转子速度
        } else {
            // 速度控制模式下，只设置速度指令
            cmd.q = 0.0f;
            cmd.dq = desired_velocity * gear_ratio;  // 速度需要乘以减速比，因为cmd.dq是转子速度
        }
        cmd.tau = 0.0f;

        // 发送命令并接收数据
        if (!serial.sendRecv(&cmd, &data)) {
            if (control_mode == ControlMode::POSITION) {
                retry_count++;
                if (retry_count >= MAX_RETRY_COUNT) {
                    std::cerr << "\nError: Failed to send command after " << MAX_RETRY_COUNT << " attempts!" << std::endl;
                    break;
                }
                need_resend = true;
                std::cerr << "\rCommunication failed, will retry in next period. Retry: " << retry_count << "/" << MAX_RETRY_COUNT << std::flush;
                std::this_thread::sleep_for(std::chrono::microseconds(RETRY_DELAY_US));
                continue;
            } else {
                // 速度控制模式下，如果通信失败，直接继续下一次循环
                std::cerr << "\rCommunication failed, continuing..." << std::flush;
                std::this_thread::sleep_for(std::chrono::microseconds(RETRY_DELAY_US));
                continue;
            }
        } else {
            // 通信成功，更新状态
            if (control_mode == ControlMode::POSITION) {
                need_resend = false;
                last_successful_time = elapsed_time;
                last_successful_position = desired_angle_deg;
                retry_count = 0;
            }
        }

        // 更新控制时间
        last_control_time = current_time;

        // 获取扭矩传感器数据
        bool has_new_data = false;
        {
            std::lock_guard<std::mutex> lock(shared_data.mutex);
            if (shared_data.has_new_data) {
                last_valid_sensor_torque = shared_data.sensor_torque;
                has_new_data = true;
            }
            shared_data.has_new_data = false;
        }

        // 计算功率
        float power = data.tau * data.dq;

        // 计算实际输出轴位置和速度（考虑减速比和零位）
        float output_position = (data.q - zero_position) / gear_ratio;
        float output_velocity = data.dq / gear_ratio;

        // 在速度控制模式下，计算期望位置
        float desired_position = 0.0f;
        if (control_mode == ControlMode::VELOCITY) {
            desired_position = calculatePosition(elapsed_time, amplitude_or_velocity, frequency_or_target);
        } else {
            desired_position = desired_angle_rad;
        }

        // 保存数据
        saveDataToFile(data_file, 
                      elapsed_time,
                      cmd.tau,
                      data.tau,
                      output_velocity,
                      output_position,
                      desired_position,
                      power,
                      last_valid_sensor_torque);

        // 打印状态（降低打印频率，每1000次打印一次）
        static int print_counter = 0;
        if (++print_counter >= 1000) {
            print_counter = 0;
            std::cout << "\rTime: " << elapsed_time << "s / " << run_time << "s"
                      << " | Position: " << output_position * (180.0f / M_PI) << " deg"
                      << " | Velocity: " << output_velocity << " rad/s"
                      << " | Desired Velocity: " << desired_velocity << " rad/s"
                      << " | Torque: " << data.tau << " Nm"
                      << " | Sensor Torque: " << last_valid_sensor_torque << " Nm"
                      << " | Temp: " << data.temp << " C"
                      << " | Error: " << data.merror 
                      << " | Resend: " << (need_resend ? "Yes" : "No")
                      << " | Retry: " << retry_count << "/" << MAX_RETRY_COUNT
                      << " | Control Freq: " << (1000000.0f / time_since_last_control) << " Hz" << std::flush;
        }

        // 更新运行时间
        elapsed_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - start_time
        ).count();

        // 检查是否达到目标时间
        if (elapsed_time >= run_time) {
            std::cout << "\nReached target run time: " << run_time << " seconds" << std::endl;
            break;
        }
    }

    std::cout << "\nMotor control completed. Data saved to " << full_path << std::endl;
    data_file.close();

    // 发送速度为0的指令，确保电机安全停止
    std::cout << "Sending zero velocity command to stop motor..." << std::endl;
    cmd.dq = 0.0f;  // 设置速度为0
    cmd.q = 0.0f;   // 设置位置为0
    cmd.tau = 0.0f; // 设置力矩为0
    
    // 发送停止指令多次，确保电机收到
    for(int i = 0; i < 10; i++) {
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Warning: Failed to send stop command!" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1ms延时
    }
    std::cout << "Motor stopped." << std::endl;

    // 绘制数据图表
    plotData(filename);

    // 停止扭矩传感器线程
    running = false;
    if (sensor_connected && sensor_thread.joinable()) {
        sensor_thread.join();
    }

    // 关闭扭矩传感器
    if (sensor_connected) {
        torqueSensor.close();
    }

    // 再次打印测试参数作为总结
    std::cout << "\nTest Summary:" << std::endl;
    printTestParameters(control_mode, amplitude_or_velocity, frequency_or_target, 
                       cycles, current, filename, run_time);

    return 0;
} 