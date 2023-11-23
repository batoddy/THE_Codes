/*
 * DCC_Variables.c
 *
 *  Created on: Jul 12, 2023
 *      Author: Batuhan
 */
#include "main.h"

int thrust;
float servo1_angle;
float servo2_angle;
float servo3_angle;
float servo4_angle;

uint8_t stabilization_flag = 0; // 0 is stable | 1 is unstable
uint8_t recovery_command = 0; // 1 is the order of the recovery
