/*
 * DCC_Motor_Control.h
 *
 *  Created on: Jul 13, 2023
 *      Author: Batuhan
 */

#ifndef INC_DCC_MOTOR_CONTROL_H_
#define INC_DCC_MOTOR_CONTROL_H_

#include "stdio.h"

void init_Servo();

void servo1_pwm(float angle);
void servo2_pwm(float angle);
void servo3_pwm(float angle);
void servo4_pwm(float angle);
void servo4_pwm_raw(uint32_t angle);

void servo_movement_control();  

void servo_reset_state();

void init_DC_Motor();

void configurate_ESC();

void DC_motor_thrust(float percentage);
void DC_motor_thrust_raw(float percentage);
void DC_motor_shutdown();
void system_emergency_shutdown();

//void ESC1_pwm_scan();

long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif /* INC_DCC_MOTOR_CONTROL_H_ */
