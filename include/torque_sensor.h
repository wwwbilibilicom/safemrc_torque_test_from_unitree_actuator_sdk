#ifndef TORQUE_SENSOR_H
#define TORQUE_SENSOR_H

#include <string>
#include <serial/serial.h>
#include <memory>

class TorqueSensor {
public:
    TorqueSensor();
    ~TorqueSensor();

    // 初始化传感器
    bool initialize(const std::string& port_name = "/dev/ttyUSB1", int baudrate = 115200);
    
    // 读取传感器数据
    double readTorque();
    
    // 检查传感器是否已初始化
    bool isInitialized() const;
    
    // 关闭传感器连接
    void close();

private:
    std::unique_ptr<serial::Serial> rs485Serial_;
    bool initialized_;
    
    // 将16位十六进制数据转换为十进制
    int16_t hex2dec(uint16_t hexData);
};

#endif // TORQUE_SENSOR_H 