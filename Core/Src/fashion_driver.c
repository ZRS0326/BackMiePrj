#include "fashion_driver.h"

uint8_t packet[FRAMESIZE];

/**
 * @brief 计算校验和
 * @param data 数据指针
 * @param length 数据长度
 * @return 校验和值
 */
uint8_t fashion_calculate_checksum(uint8_t *data, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return (uint8_t)sum ;
}

/**
 * @brief 发送PING指令
 * @param servo_id 舵机ID (0-254)
 * @return HAL状态
 */
void fashion_send_ping(uint8_t servo_id)
{
    // 构建数据包
    packet[0] = FASHION_FRAME_HEADER_REQ_1;  // 帧头1
    packet[1] = FASHION_FRAME_HEADER_REQ_2;  // 帧头2
    packet[2] = FASHION_CMD_PING;            // 指令码
    packet[3] = 0x01;                         // 内容长度
    packet[4] = servo_id;                    // 舵机ID
    
    // 计算校验和（帧头到内容部分）
    packet[5] = fashion_calculate_checksum(packet, 5);
    
    // 使用DMA发送数据
    HAL_UART_Transmit_DMA(&huart1, packet, 6);
}

/**
 * @brief 发送单圈角度控制指令
 * @param servo_id 舵机ID (0-254)
 * @param angle 目标角度（单位：0.1度，例如90.0度 = 900）
 * @param time_ms 运动时间（单位：毫秒）
 * @return HAL状态
 */
void fashion_send_single_angle(uint8_t servo_id, int16_t angle, uint16_t time_ms)
{
    // 构建数据包
    packet[0] = FASHION_FRAME_HEADER_REQ_1;  // 帧头1
    packet[1] = FASHION_FRAME_HEADER_REQ_2;  // 帧头2
    packet[2] = FASHION_CMD_SINGLE_ANGLE;    // 指令码
    packet[3] = 0x07;                         // 内容长度（舵机ID + 角度 + 时间 + 功率）
    packet[4] = servo_id;                    // 舵机ID
    
    // 角度值（小端序，16位）
    packet[5] = (uint8_t)(angle & 0xFF);      // 低字节
    packet[6] = (uint8_t)((angle >> 8) & 0xFF); // 高字节
    
    // 时间值（小端序，16位）
    packet[7] = (uint8_t)(time_ms & 0xFF);    // 低字节
    packet[8] = (uint8_t)((time_ms >> 8) & 0xFF); // 高字节
    
    packet[9] = 0x00;                         // 功率
    packet[10] = 0x00;                         // 功率

    // 计算校验和（帧头到内容部分）
    packet[11] = fashion_calculate_checksum(packet, 11);
    
    // 使用DMA发送数据
    HAL_UART_Transmit_DMA(&huart1, packet, 12);
}