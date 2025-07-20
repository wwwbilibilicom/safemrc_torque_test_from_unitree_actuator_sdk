#include <unistd.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <cstdlib>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

// 电机类型定义
#define MOTOR_TYPE MotorType::GO_M8010_6
#define WORK_KP 0.0f
#define WORK_KD 0.08f   // 增大速度环增益
#define CONTROL_PERIOD_US 10 // 10us采样周期
#define MAX_RETRY_COUNT 3
#define RETRY_DELAY_US 5
#define DEFAULT_CURRENT 1.0f

// 钟形加速度曲线 a(t) = a_max * sin(pi * t / t_total)
// 角速度和角位移为其积分
// 用户输入最大速度v_max和最大位移theta_max，自动推导a_max和t_total

// 解析推导：
// v(t) = \int a(t) dt = -(a_max * t_total / pi) * cos(pi * t / t_total) + (a_max * t_total / pi)
// v_max = a_max * t_total / pi
// theta(t) = \int v(t) dt = (a_max * t_total^2 / pi^2) * (sin(pi * t / t_total) - (pi * t / t_total) * cos(pi * t / t_total)) + (a_max * t_total / pi) * t
// theta_max = (2 * a_max * t_total^2) / pi^2
//
// 联立解得：
// t_total = (pi * theta_max) / (2 * v_max)
// a_max = (pi * v_max) / t_total

float bellShapeAcceleration(float t, float t_total, float a_max) {
    if (t < 0) return 0.0f;
    if (t > t_total) return 0.0f;
    return a_max * sin(M_PI * t / t_total);
}

float bellShapeVelocity(float t, float t_total, float a_max) {
    if (t < 0) return 0.0f;
    if (t > t_total) return 0.0f;
    return -(a_max * t_total / M_PI) * cos(M_PI * t / t_total) + (a_max * t_total / M_PI);
}

float bellShapePosition(float t, float t_total, float a_max) {
    if (t < 0) return 0.0f;
    if (t > t_total) return bellShapePosition(t_total, t_total, a_max);
    // theta(t) = (a_max * t_total^2 / pi^2) * (sin(pi * t / t_total) - (pi * t / t_total) * cos(pi * t / t_total)) + (a_max * t_total / pi) * t
    float term1 = (a_max * t_total * t_total) / (M_PI * M_PI);
    float s = sin(M_PI * t / t_total);
    float c = cos(M_PI * t / t_total);
    float x = (M_PI * t / t_total);
    return term1 * (s - x * c) + (a_max * t_total / M_PI) * t;
}

// 对称加减速钟形曲线
// 用户输入最大速度v_max和加速段末端位移theta1，自动推导加速段时间t1和最大加速度a_max
// 总运动时间2*t1，总位移2*theta1

// 加速段 a(t) = a_max * sin(pi * t / t1), t in [0, t1]
// v(t) = -(a_max * t1 / pi) * cos(pi * t / t1) + (a_max * t1 / pi)
// v_max = a_max * t1 / pi
// theta1 = (a_max * t1^2) / pi
// 联立解得 t1 = pi * theta1 / v_max, a_max = v_max * pi / t1

// 减速段 a(t) = -a_max * sin(pi * (t-t1) / t1), t in [t1, 2*t1]
// v(t) = v_max - (a_max * t1 / pi) * (1 - cos(pi * (t-t1) / t1))
// theta(t) = theta1 + v_max * (t-t1) - (a_max * t1^2 / pi^2) * (sin(pi * (t-t1) / t1) - (pi * (t-t1) / t1) * cos(pi * (t-t1) / t1))

float bellShapeAccelerationSym(float t, float t1, float a_max) {
    if (t < 0) return 0.0f;
    if (t < t1) {
        return a_max * sin(M_PI * t / t1);
    } else if (t < 2 * t1) {
        return -a_max * sin(M_PI * (t - t1) / t1);
    } else {
        return 0.0f;
    }
}

float bellShapeVelocitySym(float t, float t1, float a_max, float v_max) {
    if (t < 0) return 0.0f;
    if (t < t1) {
        return -(a_max * t1 / M_PI) * cos(M_PI * t / t1) + (a_max * t1 / M_PI);
    } else if (t < 2 * t1) {
        return v_max - (a_max * t1 / M_PI) * (1 - cos(M_PI * (t - t1) / t1));
    } else {
        return 0.0f;
    }
}

float bellShapePositionSym(float t, float t1, float a_max, float v_max, float theta1) {
    if (t < 0) return 0.0f;
    if (t < t1) {
        float term1 = (a_max * t1 * t1) / (M_PI * M_PI);
        float s = sin(M_PI * t / t1);
        float c = cos(M_PI * t / t1);
        float x = (M_PI * t / t1);
        return term1 * (s - x * c) + (a_max * t1 / M_PI) * t;
    } else if (t < 2 * t1) {
        float tau = t - t1;
        float term2 = (a_max * t1 * t1) / (M_PI * M_PI);
        float s2 = sin(M_PI * tau / t1);
        float c2 = cos(M_PI * tau / t1);
        float x2 = (M_PI * tau / t1);
        return theta1 + v_max * tau - term2 * (s2 - x2 * c2);
    } else {
        return 2 * theta1;
    }
}

// 保存数据到文件的函数
void saveDataToFile(std::ofstream& file, float time, float desired_acc, float desired_velocity, float desired_position, float output_velocity, float output_position) {
    file << time << "," << desired_acc << "," << desired_velocity << "," << desired_position << "," << output_velocity << "," << output_position << "\n";
}

// 获取用户输入，如果用户直接按回车则使用默认值
float getInputWithDefault(const std::string& prompt, float default_value) {
    std::string input;
    std::cout << prompt << " [" << default_value << "]: ";
    std::getline(std::cin, input);
    if (input.empty()) return default_value;
    try { return std::stof(input); } catch (...) {
        std::cout << "Invalid input, using default value: " << default_value << std::endl;
        return default_value;
    }
}

// 创建目录的函数
void ensureDataDirectory() {
    mkdir("data", 0777);
    mkdir("figure", 0777);
}

// 生成期望速度的函数（举例：正弦波速度）
float generateDesiredVelocity(float time, float amplitude, float frequency) {
    // amplitude 单位: rad/s，frequency 单位: Hz
    return amplitude * sin(2 * M_PI * frequency * time);
}

// 新加速段参数推导和波形函数
// 边界条件：
// t=0: a=0, v=0, theta=0
// t=t1: a=0, v=v_max, theta=theta1
// a(t) = a_max * sin(pi * t / t1)
// v(t) = -(a_max * t1 / pi) * cos(pi * t / t1) + (a_max * t1 / pi)
// theta(t) = (a_max * t1^2 / pi^2) * (sin(pi * t / t1) - (pi * t / t1) * cos(pi * t / t1)) + (a_max * t1 / pi) * t
// 边界条件2代入t1: v_max = 2 * a_max * t1 / pi, theta1 = (a_max * t1^2) / pi
// 联立解得：
// t1 = (pi * theta1) / v_max
// a_max = (v_max * pi) / (2 * t1)

// 重新实现参数推导，保证边界条件
void solveBellShapeParams(float v_max, float theta1, float& t1, float& a_max) {
    // t1 = (pi * theta1) / v_max
    t1 = (M_PI * theta1) / v_max;
    // a_max = (v_max * pi) / (2 * t1)
    a_max = (v_max * M_PI) / (2.0f * t1);
}

// 自动调零函数：将当前位置设为零位
void setCurrentPositionAsZero(SerialPort& serial, MotorCmd& cmd, MotorData& data, float gear_ratio, float& zero_position) {
    std::cout << "\n[Auto Zeroing] Setting current position as zero..." << std::endl;
    // 1. 发送零刚度命令
    cmd.kp = 0.0f;
    cmd.kd = 0.0f;
    cmd.tau = 0.0f;
    for(int i = 0; i < 100; i++) { // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during zeroing!" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
    // 2. 记录当前位置为零位
    zero_position = data.q;
    float output_position = zero_position / gear_ratio;
    std::cout << "[Auto Zeroing] Rotor position: " << zero_position << " rad, Output position: " << output_position << " rad (" << output_position * (180.0f / M_PI) << " deg)" << std::endl;
    // 3. 设置cmd.q为零位
    cmd.q = zero_position;
    // 4. 恢复速度环参数
    cmd.kp = WORK_KP;
    cmd.kd = WORK_KD;
    for(int i = 0; i < 100; i++) { // 等待约200ms
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "Error: Lost communication during control parameter restoration!" << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
    }
    std::cout << "[Auto Zeroing] Zero position set completed." << std::endl;
}

int main() {
    ensureDataDirectory();
    SerialPort serial("/dev/ttyUSB2");
    MotorCmd cmd;
    MotorData data;
    cmd.motorType = MOTOR_TYPE;
    data.motorType = MOTOR_TYPE;
    float gear_ratio = queryGearRatio(MOTOR_TYPE);
    std::cout << "Gear ratio: " << gear_ratio << std::endl;
    cmd.mode = queryMotorMode(MOTOR_TYPE, MotorMode::FOC);
    cmd.id = 0;
    cmd.kp = WORK_KP;
    cmd.kd = WORK_KD;
    cmd.q = 0.0f;
    cmd.dq = 0.0f;
    cmd.tau = 0.0f;
    // 测试电机通信
    std::cout << "Testing motor communication..." << std::endl;
    if (!serial.sendRecv(&cmd, &data)) {
        std::cerr << "Error: Failed to communicate with motor!" << std::endl;
        return 1;
    }
    std::cout << "Motor communication successful!" << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // 自动调零
    float zero_position = 0.0f;
    setCurrentPositionAsZero(serial, cmd, data, gear_ratio, zero_position);
    // 用户输入参数
    float v_max = getInputWithDefault("Enter max velocity (rad/s)", 5.5f);
    float theta1 = getInputWithDefault("Enter angle at max velocity (deg)", 300.0f);
    float theta1_rad = theta1 * (M_PI / 180.0f); // 转为弧度
    float run_time = getInputWithDefault("Enter total run time (s)", 10.0f); // 总运行时间

    // 用新函数推导加速段参数
    float t1 = 0.0f, a_max = 0.0f;
    solveBellShapeParams(v_max, theta1_rad, t1, a_max);
    std::cout << "Acceleration phase (bell shape): " << t1 << " s, Constant velocity phase: " << run_time - t1 << " s, Max velocity: " << v_max << " rad/s, Max acceleration: " << a_max << " rad/s^2" << std::endl;
    std::string filename;
    std::cout << "Enter filename [bellshape_velocity]: ";
    std::getline(std::cin, filename);
    if (filename.empty()) filename = "bellshape_velocity";
    std::string full_path = "../example/data/" + filename + ".csv";
    std::ofstream data_file(full_path);
    if (!data_file.is_open()) {
        std::cerr << "Error: Could not open file " << full_path << std::endl;
        return 1;
    }
    data_file << "Time(s),Desired_Acceleration(rad/s^2),Desired_Velocity(rad/s),Desired_Position(rad),Output_Velocity(rad/s),Output_Position(rad)\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    float elapsed_time = 0.0f;
    std::cout << "Starting bell-shaped acceleration + constant velocity motion..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    while (elapsed_time < run_time) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_start = std::chrono::duration<float>(current_time - start_time).count();
        if (time_since_start < elapsed_time + CONTROL_PERIOD_US / 1e6f) {
            std::this_thread::sleep_for(std::chrono::microseconds(CONTROL_PERIOD_US));
            continue;
        }
        elapsed_time = time_since_start;
        float desired_velocity = 0.0f;
        float desired_acc = 0.0f;
        float desired_pos = 0.0f;
        if (elapsed_time < t1) {
            // 第一阶段：正弦加速度（钟形曲线）
            desired_acc = bellShapeAcceleration(elapsed_time, t1, a_max);
            desired_velocity = bellShapeVelocity(elapsed_time, t1, a_max);
            desired_pos = bellShapePosition(elapsed_time, t1, a_max);
        } else {
            // 第二阶段：匀速
            desired_acc = 0.0f;
            desired_velocity = v_max;
            // 匀速阶段位置 = 加速末端位置 + 匀速段位移
            desired_pos = bellShapePosition(t1, t1, a_max) + v_max * (elapsed_time - t1);
        }
        // 只用速度环控制，位置指令保持当前实际位置
        cmd.kp = 0.0f;
        cmd.kd = WORK_KD;
        cmd.q = data.q; // 保持当前位置
        cmd.dq = desired_velocity * gear_ratio;
        cmd.tau = 0.0f;
        if (!serial.sendRecv(&cmd, &data)) {
            std::cerr << "\nError: Communication failed!" << std::endl;
            break;
        }
        float output_position = (data.q - zero_position) / gear_ratio;
        float output_velocity = data.dq / gear_ratio;
        saveDataToFile(data_file, elapsed_time, desired_acc, desired_velocity, desired_pos, output_velocity, output_position);
        static int print_counter = 0;
        if (++print_counter >= 100) {
            print_counter = 0;
            std::cout << "\rTime: " << elapsed_time << "s / " << run_time << "s"
                      << " | Desired_Velocity: " << desired_velocity << " | Output_Velocity: " << output_velocity
                      << std::flush;
        }
    }
    std::cout << "\nMotion completed. Data saved to " << full_path << std::endl;
    data_file.close();
    // 发送停止指令
    cmd.dq = 0.0f;
    cmd.q = 0.0f;
    cmd.tau = 0.0f;
    for(int i = 0; i < 10; i++) {
        serial.sendRecv(&cmd, &data);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    std::cout << "Motor stopped." << std::endl;

    // 自动绘图并弹窗显示
    std::string plot_cmd = "cd ../example && /home/wenbo/anaconda3/envs/torque-bench/bin/python plot_data.py data/" + filename + ".csv";
    int plot_result = system(plot_cmd.c_str());
    if (plot_result != 0) {
        std::cerr << "Error: Failed to generate and display plot!" << std::endl;
    }
    return 0;
} 