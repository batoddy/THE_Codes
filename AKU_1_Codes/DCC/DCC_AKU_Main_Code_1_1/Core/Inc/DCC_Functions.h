/*
 * DCC_Functions.h
 *
 *  Created on: Jul 12, 2023
 *      Author: Batuhan
 */

#ifndef INC_DCC_FUNCTIONS_H_
#define INC_DCC_FUNCTIONS_H_
#include "main.h"
#include "DCC_Variables.h"

extern IMU imu;
extern Altitude altitude;
extern Velocity velocity;
extern Time time;

extern char telemetry_str_packet [200];
extern uint8_t flight_state;


int initBarometer();
int readAltitude();

void init_IMU();
void read_IMU();

void getTime();

//------------------------------UART-----------------------------//
void send_to_LTC_via_UART();
void send_to_LTC_via_UART_test_version();
void emergency_shutdown_UART_init();
void emergency_shutdown_UART();

void give_recov_signal();

void check_stabilization_state();
void check_out_of_control();

void system_check();
#endif /* INC_DCC_FUNCTIONS_H_ */
