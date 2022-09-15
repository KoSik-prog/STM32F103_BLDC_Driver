/*
 * bldc.h
 *
 *  Created on: Apr 9, 2022
 *      Author: KoSik
 *      e-mail: kosik84@gmail.com
 *
 *     version: 1.0
 */

#ifndef INC_BLDC_H_
#define INC_BLDC_H_

struct BLDCMotorSt{
	uint32_t pwmU;
	uint32_t pwmV;
	uint32_t pwmW;
	int16_t fieldPosition;  //ustawiona pozycja pola magnetycznego dla kompletnego obrotu - 4095 pozycji
	int16_t fieldLastPosition;
	uint16_t expectedPosition; // oczekiwana pozycja silnika
	uint8_t direction; // kierunek 0-L 1-R
	double distance; // dystans od punktu oczekiwanego do aktualnego
	double speed;
	//double accelerate;
	double expectedPower;
	double actualPower;
	//double expectedAcceleration;
	double actualAcceleration;
	double actualSpeed;
	double expectedSpeed;
	double actualSpeedARRReg; //zmienna przechowujaca ustawienie ARR timera 2
	//uint16_t expectedDeadZone;  // martwa strefa
};

struct BLDCencoderSt{
	uint8_t status;
	uint16_t angle; //pozycja enkodera RAW
	uint16_t calculatedAngle;  // pozycja z enkodera po obliczeniu offsetu
	uint8_t magnetStatus;
	uint16_t offset; // offset ustawiony przy inicjalizacji
};

#define DLUGOSC_SPOCZYNKU 9
#define PWM_MAX 1023
#define MOTOR_RESOLUTION 819.0 // 5 x 819 = 4095

#define ARR_MIN 400
#define ACCELERATION_MAX 25 // ilosc krokow pola magnetycznego ktore mozna przeskoczyc jednym krokiem



void bldcCalc(TIM_HandleTypeDef *tim);
HAL_StatusTypeDef bldcInit(TIM_HandleTypeDef *tim, uint32_t channel1, uint32_t channel2, uint32_t channel3);
HAL_StatusTypeDef bldcSyncWithEncoder(uint8_t divide);
void bldcSetNewPosition(uint16_t deadZone, uint16_t stepsToChange);
uint8_t bldcCheckReachedPosition(uint16_t zeroPoint);
void bldcHapticSwitch(uint16_t point1, uint16_t point2);
void setMotorPosition(uint16_t position, float power);
uint16_t createFocPoint(uint16_t punktX);
void createFocArray(uint16_t *tab);
void setField(int16_t pozycja_silnika, float moc_silnika);
void delay_us(uint32_t delay_us);

#endif /* INC_BLDC_H_ */
