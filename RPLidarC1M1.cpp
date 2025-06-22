#include "RPLidarC1M1.h"

RPLidarC1M1::RPLidarC1M1()
    : _serial(NULL)
{
    _currentMeasurement.distance = 0;
    _currentMeasurement.angle = 0;
    _currentMeasurement.quality = 0;
    _currentMeasurement.startBit = 0;
}


RPLidarC1M1::~RPLidarC1M1()
{
    end();
}

bool RPLidarC1M1::begin(HardwareSerial& serial)
{
    if (isOpen()) {
      end(); 
    }
    _serial = &serial;
    _serial->begin(RPLIDAR_SERIAL_BAUDRATE);
}

void RPLidarC1M1::end()
{
    if (isOpen()) {
       _serial->end();
       _serial = NULL;
    }
}

bool RPLidarC1M1::isOpen()
{
    return _serial ? true : false; 
}

uint32_t RPLidarC1M1::getHealth(rplidar_response_device_health_t &healthinfo, uint32_t timeout)
{
    uint32_t currentTs = millis();
    uint32_t remainingtime;
  
    uint8_t *infobuf = (uint8_t *)&healthinfo;
    uint8_t recvPos = 0;

    rplidar_ans_header_t response_header;
    uint32_t  ans;


    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        if ((response_header.size) < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }
        
        while ((remainingtime = millis() - currentTs) <= timeout) {
            int currentbyte = _serial->read();
            if (currentbyte < 0) continue;
            
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_health_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OPERATION_TIMEOUT;
}

// ask the RPLidarC1M1 for its device info like the serial number
uint32_t RPLidarC1M1::getDeviceInfo(rplidar_response_device_info_t & info, uint32_t timeout )
{
    uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t*)&info;
    rplidar_ans_header_t response_header;
    uint32_t  ans;

    if (!isOpen()) return RESULT_OPERATION_FAIL;

    {
        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO,NULL,0))) {
            return ans;
        }

        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        if (response_header.size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        while ((remainingtime = millis() - currentTs) <= timeout) {
            int currentbyte = _serial->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(rplidar_response_device_info_t)) {
                return RESULT_OK;
            }
        }
    }
    
    return RESULT_OPERATION_TIMEOUT;
}

void RPLidarC1M1::reset() 
{
    int incomingByte = 0;

    uint8_t pkt1[2];
    pkt1[0] = 0xA5;
    pkt1[1] = 0x40;

    _serial->write((uint8_t*)&pkt1, 2);

    while (_serial->available() > 0) {
        incomingByte += _serial->read();
    }
}

void RPLidarC1M1::stop()
{
    int incomingByte = 0;

    uint8_t pkt2[6];
    pkt2[0] = 0xA5;
    pkt2[1] = 0xA8;
    pkt2[2] = 0x02;
    pkt2[3] = 0xE0;
    pkt2[4] = 0x01;
    pkt2[5] = 0xEE;

    _serial->write((uint8_t*)&pkt2, 6);

    while (_serial->available() > 0) {
        incomingByte += _serial->read();
    }

    incomingByte = 0;

    uint8_t pkt1[2];
    pkt1[0] = 0xA5;
    pkt1[1] = 0x25;

    _serial->write((uint8_t*)&pkt1, 2);

    while (_serial->available() > 0) {
        incomingByte += _serial->read();
    }
}

void RPLidarC1M1::startScan()
{
    int incomingByte = 0;

    uint8_t pkt2[6];
    pkt2[0] = 0xA5;
    pkt2[1] = 0xA8;
    pkt2[2] = 0x02;
    pkt2[3] = 0x58;
    pkt2[4] = 0x02;
    pkt2[5] = 0x55;

    _serial->write((uint8_t*)&pkt2, 6);

    while (_serial->available() > 0) {
        incomingByte += _serial->read();
    }

    incomingByte = 0;

    uint8_t pkt11[2];
    pkt11[0] = 0xA5;
    pkt11[1] = 0x20;

    _serial->write((uint8_t*)&pkt11, 2);

    while (_serial->available() > 0) {
        incomingByte += _serial->read();
    }
}

uint32_t RPLidarC1M1::waitPoint(uint32_t timeout)
{
   uint32_t currentTs = millis();
   uint32_t remainingtime;
   rplidar_response_measurement_node_t node;
   uint8_t *nodebuf = (uint8_t*)&node;

   uint8_t recvPos = 0;

   while ((remainingtime=millis() - currentTs) <= timeout) {
        int currentbyte = _serial->read();
        if (currentbyte<0) continue;

        switch (recvPos) {
            case 0: 
                {
                    uint8_t tmp = (currentbyte>>1);
                    if (!((tmp ^ currentbyte) & 0x1 )) {
                        continue;
                    } 
                }
                break;
            case 1: 
                {
                    if (!currentbyte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
          }
          nodebuf[recvPos++] = currentbyte;

          if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
              _currentMeasurement.distance = node.distance_q2/4.0f;
              _currentMeasurement.angle = (node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
              _currentMeasurement.quality = (node.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
              _currentMeasurement.startBit = (node.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);
              return RESULT_OK;
          }
        
   }

   return RESULT_OPERATION_TIMEOUT;
}

uint32_t RPLidarC1M1::_sendCommand(uint8_t cmd, const void * payload, size_t payloadsize)
{

    rplidar_cmd_packet_t pkt_header;
    rplidar_cmd_packet_t* header = &pkt_header;
    uint8_t checksum = 0;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    _serial->write((uint8_t *)header, 2);

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((uint8_t *)payload)[pos];
        }

        uint8_t sizebyte = payloadsize;
        _serial->write((uint8_t *)&sizebyte, 1);
        _serial->write((uint8_t *)&payload, sizebyte);
        _serial->write((uint8_t *)&checksum, 1);
    }

    return RESULT_OK;
}

uint32_t RPLidarC1M1::_waitResponseHeader(rplidar_ans_header_t * header, uint32_t timeout)
{
    uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *headerbuf = (uint8_t*)header;
    while ((remainingtime = millis() - currentTs) <= timeout) {
        
        int currentbyte = _serial->read();
        if (currentbyte<0) continue;
        switch (recvPos) {
        case 0:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE1) {
                continue;
            }
            break;
        case 1:
            if (currentbyte != RPLIDAR_ANS_SYNC_BYTE2) {
                recvPos = 0;
                continue;
            }
            break;
        }
        headerbuf[recvPos++] = currentbyte;

        if (recvPos == sizeof(rplidar_ans_header_t)) {
            return RESULT_OK;
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}

