#include "torque_sensor.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

TorqueSensor::TorqueSensor() : initialized_(false) {}

TorqueSensor::~TorqueSensor() {
    close();
}

bool TorqueSensor::initialize(const std::string& port_name, int baudrate) {
    try {
        rs485Serial_ = std::make_unique<serial::Serial>();
        rs485Serial_->setPort(port_name);
        rs485Serial_->setBaudrate(baudrate);
        // 增加超时时间到500ms
        serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
        rs485Serial_->setTimeout(timeout);
        rs485Serial_->open();
        
        if (!rs485Serial_->isOpen()) {
            std::cerr << "Unable to open RS485 port: " << port_name << std::endl;
            return false;
        }
        
        // 清空缓冲区
        rs485Serial_->flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        initialized_ = true;
        std::cout << "Torque sensor initialized successfully on port " << port_name << std::endl;
        return true;
    } catch (const serial::IOException& e) {
        std::cerr << "Error initializing torque sensor: " << e.what() << std::endl;
        return false;
    }
}

double TorqueSensor::readTorque() {
    if (!initialized_ || !rs485Serial_->isOpen()) {
        std::cerr << "Torque sensor not initialized or port closed" << std::endl;
        return 0.0;
    }

    try {
        // 清空接收缓冲区
        rs485Serial_->flush();
        
        // 发送读取命令
        uint8_t awake[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D};
        size_t bytes_written = rs485Serial_->write(awake, sizeof(awake));
        
        if (bytes_written != sizeof(awake)) {
            std::cerr << "Failed to write complete command" << std::endl;
            return 0.0;
        }

        // 等待响应
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 检查可用数据
        size_t available = rs485Serial_->available();
        if (available < 8) {  // 确保至少有8字节可读
            std::cerr << "Insufficient data available: " << available << " bytes" << std::endl;
            return 0.0;
        }

        // 读取所有可用数据
        uint8_t buffer[32];
        size_t bytes_read = rs485Serial_->read(buffer, available);

        // 查找帧头并解析数据
        for (size_t i = 0; i < bytes_read - 6; i++) {
            if (buffer[i] == 0x01 && buffer[i + 1] == 0x03) {
                // 使用正确的字节位置解析数据
                uint16_t U_DATA = (uint16_t)buffer[i + 6];
                U_DATA |= ((uint16_t)buffer[i + 5]) << 8;
                
                // 转换为扭矩值
                return hex2dec(U_DATA) / 100.0;
            }
        }

        std::cerr << "Valid frame header not found in response" << std::endl;
    } catch (const serial::IOException& e) {
        std::cerr << "Error reading torque sensor: " << e.what() << std::endl;
    }
    
    return 0.0;
}

bool TorqueSensor::isInitialized() const {
    return initialized_;
}

void TorqueSensor::close() {
    if (rs485Serial_ && rs485Serial_->isOpen()) {
        rs485Serial_->close();
    }
    initialized_ = false;
}

int16_t TorqueSensor::hex2dec(uint16_t hexData) {
    return (int16_t)hexData;
} 