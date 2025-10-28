/*
 * AD5242.h
 *
 *  Created on: Oct 28, 2025
 *      Author: wzp
 */

#ifndef INC_AD5242_H_
#define INC_AD5242_H_

#include "main.h"

#define AD5242_I2C_ADDR_BASE  0x2C  // A1,A0=GND -> 0101100b
#define AD5242_ADDR(a1,a0)    ((AD5242_I2C_ADDR_BASE | ((a1)<<1) | (a0)) << 1)

#define AD5242_1_ADDR	0x58
#define AD5242_2_ADDR	0x5A
#define AD5242_3_ADDR	0x5C
#define AD5242_4_ADDR	0x5E
#define AD5242_Channel_A	0
#define AD5242_Channel_B	1

HAL_StatusTypeDef AD5242_Write(I2C_HandleTypeDef *hi2c,
                               uint8_t addr,
                               uint8_t channel,
                               uint8_t data);
HAL_StatusTypeDef AD5242_Read(I2C_HandleTypeDef *hi2c,
                              uint8_t addr,
							  uint8_t channel,
                              uint8_t *data);

#endif /* INC_AD5242_H_ */
