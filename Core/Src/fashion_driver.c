#include "fashion_driver.h"

uint8_t packet[FRAMESIZE];
uint16_t angle_read;

/**
 * @brief 处理舵机回包数据
 * @param length 数据包长度
 * @return 是否成功处理
 */
void fashion_process_response(uint8_t length)
{    
    // 检查帧头是否正确
    if (recv_frame1[0] == FASHION_FRAME_HEADER_RESP_1 && recv_frame1[1] == FASHION_FRAME_HEADER_RESP_2)
    {      
    // 根据命令ID处理不同类型的回包
    switch (recv_frame1[2])
    {
        case FASHION_CMD_READ_ANGLE: // 角度读取回包 (0x0A)
						angle_read = ((uint16_t)recv_frame1[6] << 8) | recv_frame1[5];                
						flag_fashion = Release;
            break;
        case FASHION_CMD_SINGLE_ANGLE: // 角度控制回包 (0x08)  
						if (recv_frame1[5] == 0x01){                  
							flag_fashion = Release;
						}
            break;
        case FASHION_CMD_PING: // 通讯检测回包 (0x01)
						HAL_UART_Transmit_IT(&huart2, recv_frame1, 6);
            break;
        case FASHION_CMD_DATA_READ: // 数据读取回包 (0x03)
            // 将回包转发到uart2
            HAL_UART_Transmit_IT(&huart2, recv_frame1, length);
            break;
        case FASHION_CMD_DATA_MONITOR: // 数据监控回包 (0x16)
            // 将回包转发到uart2
            HAL_UART_Transmit_IT(&huart2, recv_frame1, length);
            break;
        default:
            break;
    }
	}
}

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

/**
 * @brief 读取舵机当前角度
 * @param servo_id 舵机ID (0-254)
 * @note 发送读取命令后，需要通过串口接收中断处理返回数据
 */
void fashion_read_servo_angle(uint8_t servo_id)
{
    // 构建数据包（根据协议规范：帧头+命令ID+长度+舵机ID+校验和）
    packet[0] = FASHION_FRAME_HEADER_REQ_1;  // 帧头1 (0x12)
    packet[1] = FASHION_FRAME_HEADER_REQ_2;  // 帧头2 (0x4C)
    packet[2] = FASHION_CMD_READ_ANGLE;      // 指令码 (0x0A - 单圈当前角度读取)
    packet[3] = 0x01;                        // 内容长度 (1个字节)
    packet[4] = servo_id;                    // 舵机ID
    
    // 计算校验和（帧头到内容部分，共5个字节）
    packet[5] = fashion_calculate_checksum(packet, 5);
    
    // 使用DMA发送数据（总共6个字节）
    HAL_UART_Transmit_DMA(&huart1, packet, 6);
}

/**
 * @brief 发送数据读取指令
 * @param servo_id 舵机ID (0-254)
 * @param data_type 数据类型（0x00: 电压, 0x01: 温度, 0x02: 版本等）
 * @note 发送读取命令后，需要通过串口接收中断处理返回数据
 */
void fashion_read_data(uint8_t servo_id, uint8_t data_type)
{
    // 构建数据包（根据协议规范：帧头+命令ID+长度+舵机ID+数据类型+校验和）
    packet[0] = FASHION_FRAME_HEADER_REQ_1;  // 帧头1 (0x12)
    packet[1] = FASHION_FRAME_HEADER_REQ_2;  // 帧头2 (0x4C)
    packet[2] = FASHION_CMD_DATA_READ;       // 指令码 (0x03 - 数据读取)
    packet[3] = 0x02;                        // 内容长度 (2个字节)
    packet[4] = servo_id;                    // 舵机ID
    packet[5] = data_type;                   // 数据类型
    
    // 计算校验和（帧头到内容部分，共6个字节）
    packet[6] = fashion_calculate_checksum(packet, 6);
    
    // 使用DMA发送数据（总共7个字节）
    HAL_UART_Transmit_DMA(&huart1, packet, 7);
}

/**
 * @brief 发送数据监控指令
 * @param servo_id 舵机ID (0-254)
 * @param monitor_type 监控类型（0x00: 角度, 0x01: 电压, 0x02: 温度等）
 * @note 发送监控命令后，舵机会定期返回监控数据
 */
void fashion_monitor_data(uint8_t servo_id, uint8_t monitor_type)
{
    // 构建数据包（根据协议规范：帧头+命令ID+长度+舵机ID+监控类型+校验和）
    packet[0] = FASHION_FRAME_HEADER_REQ_1;  // 帧头1 (0x12)
    packet[1] = FASHION_FRAME_HEADER_REQ_2;  // 帧头2 (0x4C)
    packet[2] = FASHION_CMD_DATA_MONITOR;    // 指令码 (0x16 - 数据监控)
    packet[3] = 0x02;                        // 内容长度 (2个字节)
    packet[4] = servo_id;                    // 舵机ID
    packet[5] = monitor_type;                // 监控类型
    
    // 计算校验和（帧头到内容部分，共6个字节）
    packet[6] = fashion_calculate_checksum(packet, 6);
    
    // 使用DMA发送数据（总共7个字节）
    HAL_UART_Transmit_DMA(&huart1, packet, 7);
}