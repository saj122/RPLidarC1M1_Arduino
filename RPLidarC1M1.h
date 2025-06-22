#pragma once

#include "Arduino.h"
#include "RPCommands.h"
#include "RPUtils.h"

#include <stdint.h>

const uint32_t RPLIDAR_SERIAL_BAUDRATE = 460800;
const uint32_t RPLIDAR_DEFAULT_TIMEOUT = 500;

struct RPLidarMeasurement
{
    float distance;
    float angle;
    uint8_t quality;
    bool  startBit;
};

class RPLidarC1M1
{
public:
    RPLidarC1M1();
    ~RPLidarC1M1();

    bool begin(HardwareSerial &serial);
    void end();
    bool isOpen(); 

    uint32_t getHealth(rplidar_response_device_health_t & healthinfo, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);
    uint32_t getDeviceInfo(rplidar_response_device_info_t & info, uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);
    void stop();
    void reset();
    uint32_t waitPoint(uint32_t timeout = RPLIDAR_DEFAULT_TIMEOUT);
    void startScan();
        
    const RPLidarMeasurement & getCurrentPoint()
    {
        return _currentMeasurement;
    }

    private:
        uint32_t _sendCommand(uint8_t cmd, const void * payload, size_t payloadsize);
        uint32_t _waitResponseHeader(rplidar_ans_header_t * header, uint32_t timeout);

    private:
        HardwareSerial* _serial;  
        RPLidarMeasurement _currentMeasurement;
};