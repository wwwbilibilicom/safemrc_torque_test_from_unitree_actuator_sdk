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

#define WORK_KP 0.0f  // 力矩控制时位置环增益为0
#define WORK_KD 0.0f  // 力矩控制时速度环增益为0

// 生成正弦波轨迹的函数
float generateSineWave(float time, float amplitude, float frequency) {
    return amplitude * sin(2 * M_PI * frequency * time);
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
    cmd.mode = queryMotorMode(MotorType::B1, MotorMode::FOC);
    cmd.kp = 0.0f;  // 设置位置环增益为0
    cmd.kd = 0.0f;  // 设置速度环增益为0
    cmd.tau = 0.0f; // 设置力矩为0
    
    // 等待电机稳定
    for(int i = 0; i < 100; i++) {  // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during zero position setting!" << std::endl;
            return;
        }
        usleep(2000);  // 2000微秒延时
    }
    
    // 2. 记录当前位置作为新的零位（考虑减速比）
    zero_position = data.q;  // 记录转子位置作为零位
    float output_position = zero_position / gear_ratio;  // 转换为输出轴位置
    std::cout << "Current rotor position: " << zero_position << " rad" << std::endl;
    std::cout << "Current output position: " << output_position << " rad ("
              << output_position * (180.0f / M_PI) << " degrees)" << std::endl;
    
    cmd.q = zero_position;  // 设置当前位置为零位

    // 3. 恢复正常控制参数（力矩控制时保持kp和kd为0）
    cmd.kp = WORK_KP;  // 力矩控制时位置环增益为0
    cmd.kd = WORK_KD;  // 力矩控制时速度环增益为0
    
    // 4. 等待电机稳定在新的控制参数下
    for(int i = 0; i < 100; i++) {  // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during control parameter restoration!" << std::endl;
            return;
        }
        usleep(2000);  // 2000微秒延时
    }
    
    std::cout << "Zero position setting completed" << std::endl;
}

// 修改绘图函数
void plotData(const std::string& filename) {
    // 使用conda的Python
    std::string command = "cd ../example && /home/wenbo/anaconda3/envs/torque-bench/bin/python plot_data.py data/" + filename;
    std::cout << "\nGenerating plots..." << std::endl;
    int result = system(command.c_str());
    if (result != 0) {
        std::cerr << "Error: Failed to generate plots!" << std::endl;
    }
}

// 创建目录的函数
void ensureDataDirectory() {
    mkdir("data", 0777);
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
        std::this_thread::sleep_for(std::chrono::microseconds(2000)); // 500Hz to match motor control
    }
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
    cmd.motorType = MotorType::B1;
    data.motorType = MotorType::B1;

    // 获取减速比
    float gear_ratio = queryGearRatio(MotorType::B1);
    std::cout << "Gear ratio: " << gear_ratio << std::endl;

    // 设置电机控制参数
    cmd.mode = queryMotorMode(MotorType::B1, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = 0.0f;  // 力矩控制时位置环增益为0
    cmd.kd = 0.0f;  // 力矩控制时速度环增益为0
    cmd.q = 0.0f;   // 位置
    cmd.dq = 0.0f;  // 速度
    cmd.tau = 0.0f; // 力矩

    // 测试电机通信
    std::cout << "Testing motor communication..." << std::endl;
    if (!serial.sendRecv(&cmd, &data)) {
        std::cerr << "Error: Failed to communicate with motor!" << std::endl;
        return 1;
    }
    std::cout << "Motor communication successful!" << std::endl;

    // 清除输入缓冲区
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // 在开始正弦波运动之前，设置当前位置为零位
    float zero_position = 0.0f;  // 用于存储零位位置
    setCurrentPositionAsZero(serial, cmd, data, gear_ratio, zero_position);

    // 获取用户输入参数（带默认值）
    float amplitude = getInputWithDefault("Enter torque amplitude (Nm)", 1.0f);
    float frequency = getInputWithDefault("Enter frequency (Hz)", 1.0f);
    float cycles = getInputWithDefault("Enter number of cycles", 3.0f);
    
    // 修改文件保存路径
    std::string filename;
    std::cout << "Enter filename [demo]: ";
    std::getline(std::cin, filename);
    if (filename.empty()) {
        filename = "demo";
    }
    
    // 构建完整的文件路径（相对于example目录）
    std::string full_path = "../example/data/" + filename + ".csv";
    
    // 计算运行时间
    float run_time = cycles / frequency;
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

    // 初始化共享数据和线程控制
    SharedData shared_data;
    std::atomic<bool> running(true);
    double last_valid_sensor_torque = 0.0;  // 保存最后一次有效的传感器数据

    // 只有在传感器连接成功时才启动传感器线程
    std::thread sensor_thread;
    if (sensor_connected) {
        sensor_thread = std::thread(torqueSensorThread, std::ref(running), std::ref(shared_data), std::ref(torqueSensor));
    }

    // 主控制循环
    while (elapsed_time < run_time) {
        // 计算期望力矩（正弦波）
        float desired_torque = generateSineWave(elapsed_time, amplitude, frequency);

        // 更新电机命令
        cmd.q = zero_position;  // 保持位置不变
        cmd.dq = 0.0f;         // 速度设为0
        cmd.tau = desired_torque; // 设置期望力矩

        // 发送命令并接收数据
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication with motor!" << std::endl;
            break;
        }

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

        // 保存数据（无论是否有新的传感器数据都保存）
        saveDataToFile(data_file, 
                      elapsed_time,
                      cmd.tau,
                      data.tau,
                      data.dq / gear_ratio,
                      (data.q-zero_position) / gear_ratio,
                      0.0f,
                      power,
                      last_valid_sensor_torque);  // 使用最后一次有效的传感器数据

        // 打印状态
        std::cout << "\rTime: " << elapsed_time << "s / " << run_time << "s"
                  << " | Position: " << ((data.q-zero_position) / gear_ratio) * (180.0f / M_PI) << " deg"
                  << " | Velocity: " << data.dq / gear_ratio << " rad/s"
                  << " | Torque: " << data.tau << " Nm"
                  << " | Sensor Torque: " << last_valid_sensor_torque << " Nm"
                  << " | Temp: " << data.temp << " C"
                  << " | Error: " << data.merror << std::flush;

        // 更新运行时间
        elapsed_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - start_time
        ).count();

        // 控制循环延时
        usleep(2000);  // 2000微秒延时，对应500Hz
    }

    std::cout << "\nMotor control completed. Data saved to " << full_path << std::endl;
    data_file.close();

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

    return 0;
} 