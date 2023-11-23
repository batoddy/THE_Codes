/*
 * DCC_Checking.c
 *
 *  Created on: Jul 14, 2023
 *      Author: Batuhan
 */
#include "DCC_Functions.h"
#include "DCC_Variables.h"

//------------------Stage Checking------------------//
uint8_t checkLiftoff() // accel & altitude
{
	if (altitude.altitude > 150)
	{
		time.flight_time_offset = time.current;
		return 1;
	}
	else
		return 0;
}
uint8_t checkBurnout() // accel & altitude
{
	return (altitude.altitude > 700 || time.flight_time > 8000 ? 1 : 0);
}
uint8_t checkApogee() // gyro & accel & altitude
{
	return ((altitude.altitude > 5500 && altitude.diff_to_max > 20) || time.flight_time > 37000 ? 1 : 0);
}
// uint8_t checkSeperationfromRocket() // altitude & accel ?
//{
// }
uint8_t checkSeperationfromContainer()
{
	if (stabilization_flag == STABLE && altitude.altitude < 210)
	{
		return 1;
	}
	else
		return 0;
}
uint8_t checkLanding() //
{
	return (altitude.altitude < 10 && velocity.vertical_velocity < 2 ? 1 : 0);
}

/*int temp_time;

uint8_t check_stabilization(){
   if(stabilization_flag == LOST_OF_CONTROL && flight_stage > FREEFALL_STAGE){

	   temp_time = time.current;
	   if(temp_time = time.current){

	   }
	   flight_state == LOST_OF_CONTROL_STAGE;

   }
}*/
