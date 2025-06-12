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

// 电机相关宏定义
#define MOTOR_TYPE MotorType::B1  // 电机型号，可选 A1、B1、C1、Go2
#define WORK_KP 0.0f             // 力矩控制时位置环增益为0
#define WORK_KD 0.0f             // 力矩控制时速度环增益为0
#define CONTROL_PERIOD_US 10     // 控制周期：10微秒 (100kHz)
#define MAX_RETRY_COUNT 3        // 最大重试次数
#define RETRY_DELAY_US 5         // 重试延时（微秒）
#define TORQUE_SMOOTHING_FACTOR 0.8f  // 力矩平滑因子 (0-1之间，越小越平滑)
#define DEFAULT_CURRENT 0.5f     // 默认安全电流值（A）

// 生成带偏置的正弦波函数
float generateBiasSinWave(float time, float bias, float amplitude, float frequency, 
                         float ramp_time, float hold_time) {
    // 计算各阶段时间
    float sine_start_time = ramp_time + hold_time;
    
    if (time < ramp_time) {
        // 使用 smoothstep 曲线实现平滑上升
        float ratio = time / ramp_time;
        return bias * (3 * ratio * ratio - 2 * ratio * ratio * ratio);
    } else if (time < sine_start_time) {
        // 保持偏置值阶段
        return bias;
    } else {
        // 正弦波阶段，从正峰值开始（余弦从1开始）
        float sine_time = time - sine_start_time;
        return bias + amplitude * cos(2 * M_PI * frequency * sine_time) - amplitude;
    }
}

// 添加数据平滑函数
float smoothData(float current_value, float new_value, float smoothing_factor = TORQUE_SMOOTHING_FACTOR) {
    return current_value * smoothing_factor + new_value * (1.0f - smoothing_factor);
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

// 生成自动文件名
std::string generateFileName(float bias, float amplitude, float frequency, 
                           float ramp_time, float hold_time, float cycles, float current) {
    std::stringstream ss;
    ss << "compositeLoad_bias" << std::fixed << std::setprecision(2) << bias;
    std::string str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_amp" << amplitude;
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_freq" << frequency;
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_ramp" << ramp_time;
    str = ss.str();
    std::replace(str.begin(), str.end(), '.', '_');
    
    ss.str("");
    ss << str << "_hold" << hold_time;
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

// 打印测试参数
void printTestParameters(float bias, float amplitude, float frequency, 
                       float ramp_time, float hold_time, float cycles, 
                       float current, const std::string& filename, float run_time) {
    std::cout << "\n========== Test Parameters ==========\n"
              << "Test Type: Composite Load Control\n"
              << "Bias Torque: " << bias << " Nm\n"
              << "Sine Wave Amplitude: " << amplitude << " Nm\n"
              << "Frequency: " << frequency << " Hz\n"
              << "Ramp Time: " << ramp_time << " s\n"
              << "Hold Time: " << hold_time << " s\n"
              << "Sine Wave Cycles: " << cycles << "\n"
              << "Control Current: " << current << " A\n"
              << "Total Run Time: " << run_time << " s\n"
              << "Data File: " << filename << ".csv\n"
              << "===================================\n" << std::endl;
}

// 设置电机零力矩的函数
void setZeroTorque(SerialPort& serial, MotorCmd& cmd, MotorData& data) {
    std::cout << "\nSetting motor to zero torque mode..." << std::endl;
    
    // 将电机设置为零力矩模式
    cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode::FOC);
    cmd.kp = 0.0f;   // 设置位置环增益为0
    cmd.kd = 0.0f;   // 设置速度环增益为0
    cmd.tau = 0.0f;  // 设置力矩为0
    
    // 发送零力矩命令多次以确保执行
    for(int i = 0; i < 100; i++) {  // 持续发送约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Warning: Communication failed while setting zero torque!" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
    
    std::cout << "Motor set to zero torque mode successfully" << std::endl;
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

    // 设置电机类型
    cmd.motorType = MOTOR_TYPE;
    data.motorType = MOTOR_TYPE;

    // 获取减速比
    float gear_ratio = queryGearRatio(MOTOR_TYPE);
    std::cout << "Motor Type: " << static_cast<int>(MOTOR_TYPE) << std::endl;
    std::cout << "Gear ratio: " << gear_ratio << std::endl;

    // 设置电机控制参数（力矩控制模式）
    cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = WORK_KP;   // 力矩控制时位置环增益为0
    cmd.kd = WORK_KD;   // 力矩控制时速度环增益为0
    cmd.q = 0.0f;       // 位置不控制
    cmd.dq = 0.0f;      // 速度不控制
    cmd.tau = 0.0f;     // 初始力矩为0

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
    float bias = getInputWithDefault("Enter bias torque (Nm)", 5.0f);
    float amplitude = getInputWithDefault("Enter sine wave amplitude (Nm)", 2.0f);
    float frequency = getInputWithDefault("Enter frequency (Hz)", 0.5f);
    float ramp_time = getInputWithDefault("Enter ramp time (s)", 2.0f);
    float hold_time = getInputWithDefault("Enter hold time (s)", 1.0f);
    float cycles = getInputWithDefault("Enter number of sine wave cycles", 3.0f);
    float current = getInputWithDefault("Enter control current (A)", DEFAULT_CURRENT);
    
    // 计算总运行时间（上升时间 + 保持时间 + 正弦波时间）
    float sine_time = cycles / frequency;
    float run_time = ramp_time + hold_time + sine_time;
    
    // 修改文件名生成函数
    std::string filename;
    std::string auto_filename = generateFileName(bias, amplitude, frequency, 
                                               ramp_time, hold_time, cycles, current);
    std::cout << "Enter filename [" << auto_filename << "]: ";
    std::getline(std::cin, filename);
    if (filename.empty()) {
        filename = auto_filename;
    }
    
    // 构建完整的文件路径（相对于example目录）
    std::string full_path = "../example/data/" + filename + ".csv";
    
    // 打印测试参数
    printTestParameters(bias, amplitude, frequency, 
                       ramp_time, hold_time, cycles, 
                       current, filename, run_time);
    
    std::cout << "\nRunning for " << run_time << " seconds" << std::endl;

    // 创建数据文件
    std::ofstream data_file(full_path);
    if (!data_file.is_open()) {
        std::cerr << "Error: Could not open file " << full_path << std::endl;
        return 1;
    }
    // 确保列名与plot_data.py中的预期匹配
    data_file << "Time(s),d_Torque(Nm),a_Torque(Nm),Velocity(rad/s),Position(rad),Desired_Position(rad),Power(W),Sensor_Torque(Nm)\n";

    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_control_time = start_time;
    float elapsed_time = 0.0f;

    // 初始化力矩平滑
    float smoothed_torque = 0.0f;

    // 初始化共享数据和线程控制
    SharedData shared_data;
    std::atomic<bool> running(true);
    double last_valid_sensor_torque = 0.0;  // 保存最后一次有效的传感器数据

    // 只有在传感器连接成功时才启动传感器线程
    std::thread sensor_thread;
    if (sensor_connected) {
        sensor_thread = std::thread(torqueSensorThread, std::ref(running), std::ref(shared_data), std::ref(torqueSensor));
    }

    std::cout << "Starting motor control..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // 主控制循环
    while (elapsed_time < run_time) {
        // 计算时间间隔
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last_control = std::chrono::duration_cast<std::chrono::microseconds>(
            current_time - last_control_time
        ).count();
        
        // 控制周期控制
        if (time_since_last_control < CONTROL_PERIOD_US) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(CONTROL_PERIOD_US - time_since_last_control)
            );
            continue;
        }
        
        // 更新控制时间
        last_control_time = current_time;

        // 计算期望力矩并进行平滑处理
        float raw_torque = generateBiasSinWave(elapsed_time, bias, amplitude, 
                                             frequency, ramp_time, hold_time);
        smoothed_torque = smoothData(smoothed_torque, raw_torque);
        
        // 更新电机命令（直接设置输出力矩）
        cmd.tau = smoothed_torque;

        // 发送命令并接收数据
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "\nError: Lost communication with motor!" << std::endl;
            break;
        }

        // 获取扭矩传感器数据
        {
            std::lock_guard<std::mutex> lock(shared_data.mutex);
            if (shared_data.has_new_data) {
                last_valid_sensor_torque = shared_data.sensor_torque;
                shared_data.has_new_data = false;
            }
        }

        // 计算功率
        float power = data.tau * data.dq;

        // 计算实际输出轴位置和速度（考虑减速比和零位）
        float output_position = (data.q - zero_position) / gear_ratio;
        float output_velocity = data.dq / gear_ratio;

        // 保存数据（保持与plot_data.py预期格式一致）
        saveDataToFile(data_file, 
                      elapsed_time,    // Time(s)
                      cmd.tau,         // d_Torque(Nm) - 期望力矩
                      data.tau,        // a_Torque(Nm) - 实际力矩
                      output_velocity, // Velocity(rad/s)
                      output_position, // Position(rad)
                      0.0f,           // Desired_Position(rad) - 力矩控制模式下不使用
                      power,          // Power(W)
                      last_valid_sensor_torque);  // Sensor_Torque(Nm)

        // 打印状态
        std::cout << "\rTime: " << elapsed_time << "s / " << run_time << "s"
                  << " | Position: " << output_position * (180.0f / M_PI) << " deg"
                  << " | Velocity: " << output_velocity << " rad/s"
                  << " | Desired Torque: " << cmd.tau << " Nm"
                  << " | Actual Torque: " << data.tau << " Nm"
                  << " | Sensor Torque: " << last_valid_sensor_torque << " Nm"
                  << " | Temp: " << data.temp << " C"
                  << " | Error: " << data.merror 
                  << " | Control Freq: " << (1000000.0f / time_since_last_control) << " Hz" << std::flush;

        // 更新运行时间
        elapsed_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - start_time
        ).count();
    }

    std::cout << "\nMotor control completed. Data saved to " << full_path << std::endl;
    data_file.close();

    // 设置电机为零力矩模式
    setZeroTorque(serial, cmd, data);

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
    printTestParameters(bias, amplitude, frequency, 
                       ramp_time, hold_time, cycles, 
                       current, filename, run_time);

    return 0;
} 