#ifndef __FASHION_DRIVER_H
#define __FASHION_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <string.h>

// 协议常量定义
#define FASHION_FRAME_HEADER_REQ_1    0x12
#define FASHION_FRAME_HEADER_REQ_2    0x4C
#define FASHION_FRAME_HEADER_RESP_1   0x05
#define FASHION_FRAME_HEADER_RESP_2   0x1C

// 指令控制码
#define FASHION_CMD_PING              0x01
#define FASHION_CMD_DATA_READ         0x03  // 数据读取
#define FASHION_CMD_SINGLE_ANGLE      0x08
#define FASHION_CMD_READ_ANGLE        0x0A
#define FASHION_CMD_DATA_MONITOR      0x16  // 数据监控

// 最大数据包长度
#define FASHION_MAX_PACKET_LENGTH     20

// 函数声明
uint8_t fashion_calculate_checksum(uint8_t *data, uint8_t length);
void fashion_send_ping(uint8_t servo_id);
void fashion_send_single_angle(uint8_t servo_id, int16_t angle, uint16_t time_ms);
void fashion_read_servo_angle(uint8_t servo_id);
void fashion_read_data(uint8_t servo_id, uint8_t data_type);
void fashion_monitor_data(uint8_t servo_id, uint8_t monitor_type);
void fashion_process_response(uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __FASHION_DRIVER_H */