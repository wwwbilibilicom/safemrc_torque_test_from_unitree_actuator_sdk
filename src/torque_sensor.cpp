#include "torque_sensor.h"
#include <iostream>
#include <iomanip>

TorqueSensor::TorqueSensor() : initialized_(false) {}

TorqueSensor::~TorqueSensor() {
    close();
}

bool TorqueSensor::initialize(const std::string& port_name, int baudrate) {
    try {
        rs485Serial_ = std::make_unique<serial::Serial>();
        rs485Serial_->setPort(port_name);
        rs485Serial_->setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        rs485Serial_->setTimeout(timeout);
        rs485Serial_->open();
        
        if (!rs485Serial_->isOpen()) {
            std::cerr << "Unable to open RS485 port: " << port_name << std::endl;
            return false;
        }
        
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
        return 0.0;
    }

    try {
        // 发送读取命令
        uint8_t awake[] = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x02, 0xA4, 0x0D};
        rs485Serial_->write(awake, sizeof(awake));

        // 读取响应
        size_t n = rs485Serial_->available();
        if (n >= 8) {
            uint8_t buffer[32];
            n = rs485Serial_->read(buffer, n);
            
            // 查找数据帧
            for (int i = 0; i < n - 6; ++i) {
                if (buffer[i] == 0x01 && buffer[i + 1] == 0x03) {
                    uint16_t U_DATA = (uint16_t)buffer[i + 6];
                    U_DATA |= ((uint16_t)buffer[i + 5]) << 8;
                    return hex2dec(U_DATA) / 100.0;
                }
            }
        }
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