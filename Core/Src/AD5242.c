/*
 * AD5242.c
 *
 *  Created on: Oct 28, 2025
 *      Author: wzp
 */


#include "AD5242.h"

HAL_StatusTypeDef AD5242_Write(I2C_HandleTypeDef *hi2c,
                               uint8_t addr,
                               uint8_t channel,
                               uint8_t data)
{
    uint8_t array[2];
    array[0] = (channel & 0x01) << 7;  // A/B, 0->A, 1->B
    array[1] = data;              // D7-D0

    return HAL_I2C_Master_Transmit(hi2c, addr, array, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef AD5242_Read(I2C_HandleTypeDef *hi2c,
                              uint8_t addr,
							  uint8_t channel,
                              uint8_t *data)
{
	uint8_t array[1];
	array[0] = (channel & 0x01) << 7;  // A/B, 0->A, 1->B
    if (HAL_I2C_Master_Transmit(hi2c, addr, &array, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    // 先对要读取的通道发出一个写指令，但data为空，再读AD5242，此时读的为刚才空写的通道。
    return HAL_I2C_Master_Receive(hi2c, addr, data, 1, HAL_MAX_DELAY);
}
