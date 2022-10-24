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

struct BLDCMotorSt {
	uint32_t pwmU;
	uint32_t pwmV;
	uint32_t pwmW;
	uint16_t expectedPosition; // expected motor position
	double speed;
	//double accelerate;
	double expectedPower;
	double actualPower;
	//double expectedAcceleration;
	double actualAcceleration;
	double actualSpeed;
	double expectedSpeed;
	double actualSpeedARRReg; //timer 2
	//uint16_t expectedDeadZone;  // dead zone
	double distance; // distance from the expected point to the current point
	uint8_t direction; // direction 0-L 1-R
	int16_t fieldPosition; //magnetic field position for complete rotation - 4095 pozycji
	int16_t fieldLastPosition;
};

struct BLDCencoderSt {
	uint8_t status;
	uint16_t angle; //encoder position RAW
	uint16_t calculatedAngle;  // calculated encoder position
	uint8_t magnetStatus;
	uint16_t offset; // position offset
};

#define REST_DELAY 9
#define PWM_MAX 1023
#define MOTOR_RESOLUTION 819.0 // 5 x 819 = 4095

#define ARR_MIN 400
#define ACCELERATION_MAX 25 // the number of magnetic field steps that can be jumped in one step

void bldc_Calculate(TIM_HandleTypeDef *tim);
HAL_StatusTypeDef bldc_Init(TIM_HandleTypeDef *tim, uint32_t channel1,
		uint32_t channel2, uint32_t channel3);
HAL_StatusTypeDef bldc_SyncWithEncoder(uint8_t divide);
void bldc_Set_NewPosition(uint16_t deadZone, uint16_t stepsToChange);
uint8_t bldcCheckReachedPosition(uint16_t zeroPoint);
void bldcHapticSwitch(uint16_t point1, uint16_t point2);
void bldc_Set_MotorPosition(uint16_t position, float power);
uint16_t bldc_Create_FocPoint(uint16_t punktX);
void bldc_Create_FocArray(uint16_t *tab);
void set_field(int16_t pozycja_silnika, float moc_silnika);
void delay_Us(uint32_t delay_Us);

#endif /* INC_BLDC_H_ */
