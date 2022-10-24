/*
 * usb_comm.h
 *
 *  Created on: 18 paź 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#ifndef INC_USB_COMM_H_
#define INC_USB_COMM_H_

uint8_t read_variable(uint8_t *Data, char sparam[3], uint16_t *param, uint8_t array_size);
void decode_message(uint8_t *Data, uint8_t array_size);

#endif /* INC_USB_COMM_H_ */
