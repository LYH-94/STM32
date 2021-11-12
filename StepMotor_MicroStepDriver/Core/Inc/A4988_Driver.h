/*
 * A4988_Driver.h
 *
 *  Created on: Oct 6, 2021
 *      Author: LYH
 */

#ifndef INC_STEPMOTORPWM_H_
#define INC_STEPMOTORPWM_H_



#endif /* INC_STEPMOTORPWM_H_ */

#include "main.h"
#include "math.h"

#define PWM_TIMER_CLOCKS 84000000  // Timer Frequency (Hz)
#define PWM_PSC (PWM_TIMER_CLOCKS/1000000)-1  // Prescaler
#define MICRO_STEP_RESOLUTION 16  // 1=Full step, 2=Halft step, 4=Quarter step, 8=Eighth step or 16=Sixteenth step

void StepMotorDriver_Iint(TIM_HandleTypeDef *htim);
uint32_t StepMotor_Control(TIM_HandleTypeDef *htim, float angular_velocity, float degree);
void StepMotor_Stop(TIM_HandleTypeDef *htim);

