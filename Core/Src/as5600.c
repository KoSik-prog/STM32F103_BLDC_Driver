/*
 * as5600.c
 *
 *  Created on: Mar 5, 2022
 *      Author: KoSik
 */

#include "main.h"
#include "as5600.h"

HAL_StatusTypeDef as5600_WriteCommand(I2C_HandleTypeDef *hi2c,
		uint8_t conf_register_addr, uint8_t setting) {
	return HAL_I2C_Mem_Write(hi2c, AS5600_I2C_ADDRESS, conf_register_addr, 1,
			&setting, 1, 100);
}

HAL_StatusTypeDef as5600_Init(I2C_HandleTypeDef *hi2c) {
	uint8_t status;
	HAL_I2C_Init(hi2c);
	status = as5600_WriteCommand(hi2c, 0x08, 0x01); //wlacz enkoder
	if (!status) {
		HAL_Delay(50);
		status = as5600_WriteCommand(hi2c, 0x08, 0x01); //wlacz enkoder
	}
	status = as5600_WriteCommand(hi2c, 0x07, 0x00);  //slow filter x16
	HAL_Delay(10);
	return status;
}

HAL_StatusTypeDef as5600_ReadPosition(I2C_HandleTypeDef *hi2c, uint16_t *angle) {
	uint8_t status = 0;
	uint8_t temp[2];

	status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS, 0x0E, 1, temp, 2, 100);
	*angle = (temp[0] << 8) + temp[1];

	return status;
}

HAL_StatusTypeDef as5600_ReadRawPosition(I2C_HandleTypeDef *hi2c,
		uint16_t *angle) {
	uint8_t status = 0;
	uint8_t temp[2];

	status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS, 0x0C, 1, temp, 2, 100);
	*angle = (temp[0] << 8) + temp[1];

	return status;
}

HAL_StatusTypeDef as5600_ReadConfig(I2C_HandleTypeDef *hi2c, uint16_t *config) {
	uint8_t temp_posision = 0;
	uint8_t status = 0;

	status = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS, 0x07, 1, &temp_posision, 1,
			100);
	*config = temp_posision << 8;
	if (status == 0) {
		HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS, 0x08, 1, &temp_posision, 1,
				100);
		*config += temp_posision;
	}

	return status;
}

/*  Magnet Status
 *  Bits    |MD|ML|MH|
 *
 * MD - magnet was detected
 * ML - magnet too weak
 * MH - magnet too strong
 * */
uint8_t as5600_StatusMagnet(I2C_HandleTypeDef *hi2c) {
	uint8_t status = 0;

	HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDRESS, 0x0B, 1, &status, 1, 100);
	return status >> 3;
}

