#pragma once

#include <stdint.h>

#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80

#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_ANS_PKTFLAG_LOOP     0x1

typedef struct _rplidar_cmd_packet_t {
    uint8_t syncByte; 
    uint8_t cmd_flag; 
    uint8_t size;
    uint8_t data[0];
} __attribute__((packed)) rplidar_cmd_packet_t;


typedef struct _rplidar_ans_header_t {
    uint8_t  syncByte1; 
    uint8_t  syncByte2; 
    uint32_t size:30;
    uint32_t subType:2;
    uint8_t  type;
} __attribute__((packed)) rplidar_ans_header_t;