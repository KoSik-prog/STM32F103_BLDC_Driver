/*
 * as5600.h
 *
 *  Created on: Mar 5, 2022
 *      Author: KoSik
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#define AS5600_I2C_ADDRESS 0x36 << 1


HAL_StatusTypeDef as5600_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t conf_register_addr, uint8_t setting);
HAL_StatusTypeDef as5600_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef as5600_ReadPosition(I2C_HandleTypeDef *hi2c, uint16_t *angle);
HAL_StatusTypeDef as5600_ReadRawPosition(I2C_HandleTypeDef *hi2c, uint16_t *angle);
HAL_StatusTypeDef as5600_ReadConfig(I2C_HandleTypeDef *hi2c, uint16_t *config);
uint8_t as5600_StatusMagnet(I2C_HandleTypeDef *hi2c);

#endif /* INC_AS5600_H_ */
