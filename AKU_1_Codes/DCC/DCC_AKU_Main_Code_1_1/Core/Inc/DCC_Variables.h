/*
 * DCC_Variables.h
 *
 *  Created on: Jul 12, 2023
 *      Author: Batuhan
 */

#ifndef SRC_DCC_VARIABLES_H_
#define SRC_DCC_VARIABLES_H_
#include "stdio.h"
/*
 * DCC_Variables.h
 *
 *  Created on: 3 Jul 2023
 *      Author: Batuhan
 */

#ifndef INC_DCC_VARIABLES_H_
#define INC_DCC_VARIABLES_H_

#define KALMAN_MEAS_E 0.08
#define KALMAN_EST_E 0.08
#define KALMAN_Q 0.03

#define KALMAN_BARO_MEAS_E 0.1
#define KALMAN_BARO_EST_E 0.1
#define KALMAN_BARO_Q 0.05

extern float thrust;
extern float servo1_angle;
extern float servo2_angle;
extern float servo3_angle;
extern float servo4_angle;

extern uint8_t recovery_command;
extern uint8_t stabilization_flag; // 0 is stable | 1 is unstable



/*----------MS5611-VARIABLES----------*/
typedef struct Altitude
{
  float pressure;
  int16_t pressure_int_2;
  float base_pressure;
  float temperature;
  uint8_t temperature_int;
  float altitude;
  int16_t int_16_altitude;
  float max_altitude;
  float temp_altitude;
  float temp_altitude_for_velocity;
  float diff_to_max;
  float prevAltitude;
  float prevAltitudeForVelocity;

} Altitude;

typedef struct Velocity
{
  float time_diff;
  float vertical_velocity;

  float prev_time;
  float max_velocity_rocket;
  float max_velocity_payload;
  int8_t max_velocity_rocket_mach;
  int8_t max_velocity_payload_mach;
  float velocity_changing; // 1 increase | 0 decerase
} Velocity;

enum barometer_state
{
  TEMPERATURE_CALLBACK = 0,
  PRESSURE_CALLBACK

};

/*----------BNO055-VARIABLES----------*/

typedef struct Vector
{
  float x;
  float y;
  float z;
  float resultant;
} Vector;

typedef struct Euler
{
  float yaw;
  float roll;
  float pitch;
} Euler;

typedef struct IMU
{
  Vector accel;
  Vector gyro;
  Euler euler;

  Vector accel_G;

} IMU;

typedef struct IMU_offset
{
  Vector accel;
  Vector gyro;
  Vector euler;

} IMU_offset;

//-------------------Variables--------------------//
extern uint8_t flight_state;

typedef union
{
  float u32;
  char u8[4];
} float_to_u8;

typedef union
{
  int16_t u16;
  char u8[2];
} u16_to_u8;

typedef struct Time
{
  uint32_t current;
  int prevTime;
  int liftoff;
  int apogee;
  int timeDifference;
  int container_seperation;
  int flight_time;
  int flight_time_offset;
  int landing;
} Time;

enum FlightStates
{
  START,
  AFTER_LIFTOFF,
  AFTER_BURNOUT,
  // AFTER_APOGEE,
  // Seperation????
  FREEFALL_STAGE,
  CONTROLLED_FLIGHT,
  TOUCHDOWN,
  LOST_OF_CONTROL_STAGE
};

enum Errors
{
  OK,
  ROCKET_SEPERATION_ERR,
  CONTAINER_SEPERATION_ERR,
  SPIN_ERR,
  BAROMETER_ERR,
  IMU_ERR
};
enum StabilizationFlag
{
  STABLE,
  UNSTABLE,
  CONTROL_LOST
};
#endif /* INC_DCC_VARIABLES_H_ */

#endif /* SRC_DCC_VARIABLES_H_ */
