/*
 * usb_comm.c
 *
 *  Created on: 18 paź 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */
#include <main.h>
#include <stdlib.h>
#include <stdio.h>

uint16_t degree = 0;
uint16_t expectedPosition = 0;


uint8_t read_variable(uint8_t *Data, char sparam[3], uint16_t *param, uint8_t array_size){
	uint8_t buf[10];
	uint8_t q;

	for (q = 0; q < sizeof(buf); q++) { // clean buffer
		buf[q] = 0;
	}
	if (Data[1] == sparam[0] && Data[2] == sparam[1] && Data[3] == sparam[2]) {
		for (q = 0; q < sizeof(Data); q++) {
			if (Data[4 + q] == '/') {
				break;
			} else {
				buf[q] = Data[4 + q];
			}
		}
		*param = atoi((char*) buf); //variable save
		for (uint16_t i = 0; i < (array_size - (4 + q)); i++) {
			Data[i] = Data[4 + q + i];
		}
		return 1;
	} else {
		return 0;
	}
}

void decode_message(uint8_t *Data, uint8_t array_size) {
	for (uint8_t j = 0; j < 3; j++) {
		if (Data[0] == '#') {
			if (!read_variable(Data, "pos", &expectedPosition, array_size)) {
				read_variable(Data, "deg", &degree, array_size);
			}
			//------------------------------------------------------------------------

			else {
				break;
			}
		}
	}
}
