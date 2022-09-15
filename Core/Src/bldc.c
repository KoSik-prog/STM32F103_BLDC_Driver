/*
 * bldc.h
 *
 *  Created on: Apr 9, 2022
 *      Author: KoSik
 *      e-mail: kosik84@gmail.com
 *
 *     version: 1.0
 */

#include "main.h"
#include "bldc.h"
#include "stdlib.h"

struct BLDCMotorSt bldcMotor;
struct BLDCencoderSt bldcEncoder;

uint16_t focArray[(int)MOTOR_RESOLUTION];
uint8_t initPhase = 0;
//uint16_t centerPoint, centerPoint2, bufPoint;



HAL_StatusTypeDef bldcInit(TIM_HandleTypeDef *tim, uint32_t channel1, uint32_t channel2, uint32_t channel3){
	HAL_StatusTypeDef status;

	status = HAL_TIM_PWM_Start_DMA(tim, channel1, (uint32_t*)&bldcMotor.pwmU, 1);
	HAL_TIMEx_PWMN_Start(tim, channel1);
	HAL_TIM_PWM_Start_DMA(tim, channel2, (uint32_t*)&bldcMotor.pwmV, 1);
	HAL_TIMEx_PWMN_Start(tim, channel2);
	HAL_TIM_PWM_Start_DMA(tim, channel3, (uint32_t*)&bldcMotor.pwmW, 1);
	HAL_TIMEx_PWMN_Start(tim, channel3);


	initPhase = 1;

	for (bldcMotor.fieldPosition = 4095; bldcMotor.fieldPosition > 0; bldcMotor.fieldPosition--) {
		setMotorPosition(bldcMotor.fieldPosition, 20);
		delay_us(30);
		if(bldcEncoder.angle < 819){
			break;
		}
	}
	for (bldcMotor.fieldPosition = 819; bldcMotor.fieldPosition > 0; bldcMotor.fieldPosition--) {
			setMotorPosition(bldcMotor.fieldPosition, 20);
			delay_us(300);
		}
	HAL_Delay(100);
	bldcEncoder.offset = bldcEncoder.angle;
	initPhase = 0;


	return status;
}


/*
 * Update field position with encoder position
 * tested with 1kHz update
 * */
HAL_StatusTypeDef bldcSyncWithEncoder(uint8_t divide){
	if(abs(bldcMotor.fieldPosition - bldcEncoder.calculatedAngle) > 50){  //korekta pozycji silnika wzgledem enkodera
		if(abs(bldcMotor.fieldPosition - bldcEncoder.calculatedAngle) > 3000){ //jesli przeskok przez 0
			if(bldcMotor.fieldPosition > bldcEncoder.calculatedAngle){
				bldcMotor.fieldPosition = bldcEncoder.calculatedAngle - ((bldcMotor.fieldPosition - (4095 + bldcEncoder.calculatedAngle)) / divide);
			} else {
				bldcMotor.fieldPosition = bldcEncoder.calculatedAngle + (((bldcMotor.fieldPosition + 4095) - bldcEncoder.calculatedAngle) / divide);
			}
		} else {
			bldcMotor.fieldPosition = bldcEncoder.calculatedAngle + ((bldcMotor.fieldPosition - bldcEncoder.calculatedAngle) / divide);
		}
	}
	return HAL_OK;
}

void bldcSetNewPosition(uint16_t deadZone, uint16_t stepsToChange) {
	if (bldcMotor.distance >= stepsToChange) {
		if (bldcMotor.direction == 0) {
			bldcMotor.fieldPosition = bldcMotor.fieldPosition + stepsToChange;
			if (bldcMotor.fieldPosition > 4095) {
				bldcMotor.fieldPosition = bldcMotor.fieldPosition - 4095;
			}
		} else {
			bldcMotor.fieldPosition = bldcMotor.fieldPosition - stepsToChange;
			if(bldcMotor.fieldPosition < 0) {
				bldcMotor.fieldPosition = 4095 - bldcMotor.fieldPosition;
			}
		}
	}
	setMotorPosition(bldcMotor.fieldPosition, bldcMotor.actualPower);
}

/*
 * obliczenie mocy i kolejnej pozycji
 * obliczenia wykonywane w cyklach 100Hz
 * */
void bldcCalc(TIM_HandleTypeDef *tim){
	uint16_t calcR, calcL;

	//  ANGLE
		if((uint16_t)bldcEncoder.angle > bldcEncoder.offset){
			bldcEncoder.calculatedAngle = (uint16_t)bldcEncoder.angle - bldcEncoder.offset;
		} else {
			bldcEncoder.calculatedAngle = (4095 + (uint16_t)bldcEncoder.angle) - bldcEncoder.offset;
		}
	// DISTANCE + DIRECTION
		if(bldcMotor.expectedPosition >= bldcMotor.fieldPosition){
			calcL = bldcMotor.expectedPosition - bldcMotor.fieldPosition;
			calcR = (4095 - bldcMotor.expectedPosition) + bldcMotor.fieldPosition;
		} else {
			calcL = (4095 - bldcMotor.fieldPosition) + bldcMotor.expectedPosition;
			calcR = bldcMotor.fieldPosition - bldcMotor.expectedPosition;
		}

		if(calcR > calcL){
			bldcMotor.distance = calcL;
			bldcMotor.direction = 0;
		} else {
			bldcMotor.distance = calcR;
			bldcMotor.direction = 1;
		}
	// SPEED
	if(bldcMotor.fieldPosition >= bldcMotor.fieldLastPosition){
		calcL = bldcMotor.fieldPosition - bldcMotor.fieldLastPosition;
		calcR = (4095 - bldcMotor.fieldPosition) + bldcMotor.fieldLastPosition;
	} else {
		calcL = (4095 - bldcMotor.fieldLastPosition) + bldcMotor.fieldPosition;
		calcR = bldcMotor.fieldLastPosition - bldcMotor.fieldPosition;
	}

	if(calcR > calcL){
		bldcMotor.speed = calcL;
	} else {
		bldcMotor.speed = calcR;
	}
	bldcMotor.fieldLastPosition = bldcMotor.fieldPosition;

	// ACCELERATION
	if(bldcMotor.speed < bldcMotor.expectedSpeed && bldcMotor.distance > 100){
		if(__HAL_TIM_GET_AUTORELOAD(tim) > ARR_MIN){
			bldcMotor.actualSpeedARRReg = bldcMotor.actualSpeedARRReg - 1;
			__HAL_TIM_SET_AUTORELOAD(tim, bldcMotor.actualSpeedARRReg);
		} else if(__HAL_TIM_GET_AUTORELOAD(tim) == ARR_MIN && bldcMotor.actualAcceleration < ACCELERATION_MAX){
			bldcMotor.actualAcceleration+=0.008;
		}
	}
	// BRAKING
	if(bldcMotor.distance < 100){
		bldcMotor.actualSpeedARRReg = 400 + (bldcMotor.distance * 6);
		__HAL_TIM_SET_AUTORELOAD(tim, bldcMotor.actualSpeedARRReg);
	}

}



/*
 * ustawic w przerwaniu 1kHz
 * */
void setMotorPosition(uint16_t position, float power){
	uint16_t pos = position;

	for(uint8_t i=0; i<20; i++){
		if(pos>=819){
			pos=pos-819;
		} else {
			break;
		}
	}
	setField(pos, power);
}

uint16_t createFocPoint(uint16_t punktX){
    float a, b;

    if(punktX > (MOTOR_RESOLUTION-1)){
        punktX -= (MOTOR_RESOLUTION-1);
    }

    if(punktX >= 0 && punktX < DLUGOSC_SPOCZYNKU){
        return PWM_MAX;
    } else if(punktX >= DLUGOSC_SPOCZYNKU && punktX < (MOTOR_RESOLUTION/2)){
        a = -PWM_MAX / ((MOTOR_RESOLUTION/2) - DLUGOSC_SPOCZYNKU);
        b = PWM_MAX - (a * DLUGOSC_SPOCZYNKU);
        return (int)((a * punktX) + b);
    } else if (punktX >= (MOTOR_RESOLUTION/2) && punktX < ((MOTOR_RESOLUTION/2) + DLUGOSC_SPOCZYNKU)){
        return 0;
    } else if (punktX >= (MOTOR_RESOLUTION/2) + DLUGOSC_SPOCZYNKU && punktX < MOTOR_RESOLUTION){
        a = ((PWM_MAX * 1.0) / (MOTOR_RESOLUTION - ((MOTOR_RESOLUTION/2) + DLUGOSC_SPOCZYNKU)));
        b = -(a * ((MOTOR_RESOLUTION/2) + DLUGOSC_SPOCZYNKU));
        return (int)((a * punktX) + b);
    } else {
        return 0;
    }
}

void createFocArray(uint16_t *tab){
    for(uint16_t q=0; q<MOTOR_RESOLUTION; q++){
        tab[q] = createFocPoint(q);
    }
}

void setField(int16_t pozycja_silnika, float moc_silnika){
	uint16_t aktualna_pozycja;

	#define przesuniecie1 MOTOR_RESOLUTION/3
	#define przesuniecie2 MOTOR_RESOLUTION/1.5

		aktualna_pozycja = pozycja_silnika;
		bldcMotor.pwmU = (uint16_t)(focArray[pozycja_silnika] * moc_silnika / 100);


		if(pozycja_silnika > MOTOR_RESOLUTION - przesuniecie1){
			aktualna_pozycja = pozycja_silnika - MOTOR_RESOLUTION + przesuniecie1;
		} else {
			aktualna_pozycja = pozycja_silnika + przesuniecie1;
		}
		bldcMotor.pwmV = (uint16_t)(focArray[aktualna_pozycja] * moc_silnika / 100);

		if(pozycja_silnika > MOTOR_RESOLUTION - przesuniecie2){
			aktualna_pozycja = pozycja_silnika - MOTOR_RESOLUTION + przesuniecie2;
		} else {
			aktualna_pozycja = pozycja_silnika + przesuniecie2;
		}
		bldcMotor.pwmW = (uint16_t)(focArray[aktualna_pozycja] * moc_silnika / 100);

}

void delay_us(uint32_t delay_us){
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}
