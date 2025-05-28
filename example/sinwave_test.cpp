#include <unistd.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <cstdlib>  // 添加此行用于system函数
#include <sys/stat.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#define WORK_KP 5.0f
#define WORK_KD 20.0f

// 生成正弦波轨迹的函数
float generateSineWave(float time, float amplitude, float frequency, float zero_offset = 0.0f) {
    return amplitude * sin(2 * M_PI * frequency * time) + zero_offset;
}

// 保存数据到文件的函数
void saveDataToFile(std::ofstream& file, float time, float desired_torque, float actual_torque, 
                   float velocity, float position, float desired_position, float power) {
    file << time << "," 
         << desired_torque << "," 
         << actual_torque << "," 
         << velocity << "," 
         << position << "," 
         << desired_position << "," 
         << power << "\n";
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

// 电机归零函数
void motorHoming(SerialPort& serial, MotorCmd& cmd, MotorData& data, float gear_ratio, float homing_time = 2.0f) {
    std::cout << "\nStarting motor homing..." << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    float elapsed_time = 0.0f;
    float initial_position = (data.q / gear_ratio) * (180.0f / M_PI);
    
    while (elapsed_time < homing_time) {
        // 使用余弦函数生成平滑的归零轨迹
        float progress = elapsed_time / homing_time;
        float desired_angle_deg = initial_position * cos(progress * M_PI / 2);
        float desired_angle_rad = desired_angle_deg * (M_PI / 180.0f);
        float rotor_angle = desired_angle_rad * gear_ratio;

        // 更新电机命令
        cmd.q = rotor_angle;
        cmd.dq = 0.0f;
        cmd.tau = 0.0f;

        // 发送命令并接收数据
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during homing!" << std::endl;
            return;
        }

        // 打印状态
        std::cout << "\rHoming progress: " << (1.0f - progress) * 100 << "%"
                  << " | Position: " << (data.q / gear_ratio) * (180.0f / M_PI) << " deg"
                  << " | Velocity: " << data.dq / gear_ratio << " rad/s"
                  << " | Torque: " << data.tau << " Nm" << std::flush;

        // 更新运行时间
        elapsed_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - start_time
        ).count();

        usleep(2000);  // 2000微秒延时，对应500Hz
    }
    std::cout << "\nMotor homing completed" << std::endl;
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

    // 3. 恢复正常控制参数
    cmd.kp = WORK_KP;  // 恢复位置环增益
    cmd.kd = WORK_KD; // 恢复速度环增益
    
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

int main() {
    // 确保data目录存在
    ensureDataDirectory();

    // 初始化串口
    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;

    // 设置电机类型为B1
    cmd.motorType = MotorType::B1;
    data.motorType = MotorType::B1;

    // 获取减速比
    float gear_ratio = queryGearRatio(MotorType::B1);
    std::cout << "Gear ratio: " << gear_ratio << std::endl;

    // 设置电机控制参数
    cmd.mode = queryMotorMode(MotorType::B1, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = 0.0f;  // 位置环增益
    cmd.kd = 0.0f; // 速度环增益
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
    float amplitude = getInputWithDefault("Enter amplitude (degrees)", 1.0f);
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
    data_file << "Time(s),d_Torque(Nm),a_Torque(Nm),Velocity(rad/s),Position(rad),Desired_Position(rad),Power(W)\n";

    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    float elapsed_time = 0.0f;

    std::cout << "Starting motor control..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // 主控制循环
    while (elapsed_time < run_time) {
        // 计算期望位置（以当前位置为基准点）
        float desired_angle_deg = generateSineWave(elapsed_time, amplitude, frequency);
        float desired_angle_rad = desired_angle_deg * (M_PI / 180.0f);
        // 将期望角度转换为转子角度，并加上零位偏移
        float rotor_angle = (desired_angle_rad * gear_ratio) + zero_position;

        // 更新电机命令
        cmd.q = rotor_angle;
        cmd.dq = 0.0f;  // 让位置控制器处理速度
        cmd.tau = 0.0f; // 不使用前馈力矩

        // 发送命令并接收数据
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication with motor!" << std::endl;
            break;
        }

        // 计算功率
        float power = data.tau * data.dq;

        // 保存数据
        saveDataToFile(data_file, 
                      elapsed_time,
                      cmd.tau,
                      data.tau,
                      data.dq / gear_ratio,
                      (data.q-zero_position) / gear_ratio,
                      desired_angle_rad,
                      power);

        // 打印状态
        std::cout << "\rTime: " << elapsed_time << "s / " << run_time << "s"
                  << " | Position: " << ((data.q-zero_position) / gear_ratio) * (180.0f / M_PI) << " deg"
                  << " | Velocity: " << data.dq / gear_ratio << " rad/s"
                  << " | Torque: " << data.tau << " Nm"
                  << " | Temp: " << data.temp << " C"
                  << " | Error: " << data.merror << std::flush;

        // 更新运行时间
        elapsed_time = std::chrono::duration<float>(
            std::chrono::high_resolution_clock::now() - start_time
        ).count();

        // 控制循环延时
        usleep(1);  // 2000微秒延时，对应500Hz
    }

    std::cout << "\nMotor control completed. Data saved to " << full_path << std::endl;
    data_file.close();

    // 执行电机归零
    motorHoming(serial, cmd, data, gear_ratio);

    // 绘制数据图表（使用完整路径）
    plotData(filename);

    return 0;
}