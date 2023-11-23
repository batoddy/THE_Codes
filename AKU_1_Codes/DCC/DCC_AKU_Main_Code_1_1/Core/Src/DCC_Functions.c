	/*
 * DCC_Functions.c
 *
 *  Created on: Jul 12, 2023
 *      Author: Batuhan
 */

#include "math.h"
#include "bno055.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "bno055_stm32.h"
#include "yrt_ms5611.h"
#include "string.h"
#include "usart.h"
#include "yrt_simpleKalman.h"

#include "File_Handling_RTOS.h"
#include "DCC_Variables.h"
#include "DCC_Motor_Control.h"
// #include "DCC_Functions.h"

IMU imu;
Altitude altitude;
Velocity velocity;
Time time;

char data_pack_final[50];
char telemetry_str_packet[200];
uint8_t flight_state;

IMU imu_offset;
uint8_t caliber_value = 250;

uint16_t accel_resultant_int16;
///////////////////////////////////////////////////////////
float prev_AccelX;
float prev_AccelY;
float prev_AccelZ;

float prev_GyroX;
float prev_GyroY;
float prev_GyroZ;

float prev_EulerRoll;
float prev_EulerPitch;
float prev_EulerYaw;
int batucuk = 0;
int batucuk2 = 0;

yrt_simpleKalman_t accelXKalman;
yrt_simpleKalman_t accelYKalman;
yrt_simpleKalman_t accelZKalman;

yrt_simpleKalman_t gyroXKalman;
yrt_simpleKalman_t gyroYKalman;
yrt_simpleKalman_t gyroZKalman;

yrt_simpleKalman_t eulerYawKalman;
yrt_simpleKalman_t eulerPitchKalman;
yrt_simpleKalman_t eulerRollKalman;

yrt_simpleKalman_t altitudeKalman;

//-----------SYSTEM-FUNCTIONS-----------//

void initFilter()
{
	yrt_simpleKalman_init(&accelXKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&accelYKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&accelZKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);

	yrt_simpleKalman_init(&gyroXKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&gyroYKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&gyroZKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);

	yrt_simpleKalman_init(&eulerYawKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&eulerPitchKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);
	yrt_simpleKalman_init(&eulerRollKalman, KALMAN_MEAS_E, KALMAN_EST_E, KALMAN_Q);

	yrt_simpleKalman_init(&altitudeKalman, KALMAN_BARO_MEAS_E,
						  KALMAN_BARO_EST_E, KALMAN_BARO_Q);
}

// float kalmanFilter(float vari, float mea_e, float est_e, float q, float last)
//{
//     float kalman_gain = 0.0, current_estimate = 0.0;
//     kalman_gain = est_e / (est_e + mea_e);
//     current_estimate = last + kalman_gain * (vari - last);
//     est_e = (1.0 - kalman_gain) * est_e + fabs(last - current_estimate) * q;
//     last = current_estimate;
//
//     return current_estimate;
// }

//-----------MS5611-----------//

static float calculateAltitude(float p, float pi)
{
	p = p / 100;
	pi = pi / 100;
	float alt = (44330 * (1.0 - pow(p / pi, 0.1903)));
	return alt;
}

int initBarometer()
{
	ms5611_init();
	ms5611_setOSR(OSR_1024);
	float temperature, pressure, ms_alt;

	HAL_Delay(100);

	for (int i = 0; i < caliber_value; i++)
		{
			// bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
			ms5611_getTemperatureAndPressure(&temperature, &pressure, &ms_alt);
			float a;
			a += pressure;
			//        altitude.basePressure += pressure;
			HAL_Delay(2);
		}

	for (int i = 0; i < caliber_value; i++)
	{
		// bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		ms5611_getTemperatureAndPressure(&temperature, &pressure, &ms_alt);
		altitude.base_pressure += pressure;
		//        altitude.basePressure += pressure;
		HAL_Delay(2);
	}

	altitude.base_pressure /= caliber_value;

	return 1;
}

int readBarometer()
{
	float temperature, pressure, ms_alt;
	ms5611_getTemperatureAndPressure(&temperature, &pressure, &ms_alt);

	altitude.pressure = pressure;
	altitude.temperature = temperature;

	return 1;
}

int readAltitude()
{
	readBarometer();

	altitude.altitude =
		calculateAltitude(altitude.pressure, altitude.base_pressure);
	altitude.altitude =
		//        kalmanFilter(altitude.altitude, 6, 4, 0.1, altitude.prevAltitude);
		yrt_simpleKalman_updateEstimate(&altitudeKalman, altitude.altitude);
	/* diif (-) ise cikiyorsun (+) ise iniyorsun  */


if(time.current > 5000){

	if (altitude.altitude > altitude.max_altitude)
	{
		altitude.max_altitude = altitude.altitude;
	}

	altitude.diff_to_max = altitude.max_altitude - altitude.altitude;

	altitude.prevAltitude = altitude.altitude;

	// calculate vertical velocity m/s
	velocity.time_diff =
		(HAL_GetTick() - velocity.prev_time) / 1000.0f;

	if (velocity.time_diff > 0.2f)
	{
		velocity.vertical_velocity =
			((altitude.altitude - altitude.prevAltitudeForVelocity) /
			 (velocity.time_diff)) /**(-1)*/;

		altitude.prevAltitudeForVelocity = altitude.altitude;
		velocity.prev_time = HAL_GetTick();
	}

	if(velocity.vertical_velocity > velocity.max_velocity_rocket){
	    	velocity.max_velocity_rocket = velocity.vertical_velocity;
	    	velocity.max_velocity_rocket_mach = (int)velocity.vertical_velocity/0.2915;
	    }

	if(velocity.vertical_velocity < velocity.max_velocity_payload){
		velocity.max_velocity_payload = velocity.vertical_velocity;
		velocity.max_velocity_payload_mach = (int)velocity.vertical_velocity/0.2915;
	}
}
	altitude.temperature_int = (uint8_t)altitude.temperature;
	altitude.pressure_int_2 = (uint16_t)altitude.pressure;
	return 1;
}

//-----------BNO055-----------//

void offset_IMU(void)
{
	static uint8_t cntr;
	for (cntr = 0; cntr < caliber_value; cntr++)
	{
		bno055_vector_t a = bno055_getVectorAccelerometer();

		bno055_vector_t b = bno055_getVectorGyroscope();

		bno055_vector_t c = bno055_getVectorEuler();
		bno055_delay(1);
	}
	// osDelay(5000);
	static uint8_t counter;
	for (counter = 0; counter < caliber_value; counter++)
	{
		bno055_vector_t accel_for_offset = bno055_getVectorAccelerometer();
		imu_offset.accel.x += accel_for_offset.x;
		imu_offset.accel.y += accel_for_offset.y;
		imu_offset.accel.z += accel_for_offset.z;

		bno055_vector_t gyro_for_offset = bno055_getVectorGyroscope();
		imu_offset.gyro.x += gyro_for_offset.x;
		imu_offset.gyro.y += gyro_for_offset.y;
		imu_offset.gyro.z += gyro_for_offset.z;

		bno055_vector_t euler_for_offset = bno055_getVectorEuler();
		imu_offset.euler.roll += euler_for_offset.x;
		imu_offset.euler.pitch += euler_for_offset.y;
		imu_offset.euler.yaw += euler_for_offset.z;
		bno055_delay(1);
	}

	imu_offset.accel.x /= caliber_value;
	imu_offset.accel.y /= caliber_value;
	imu_offset.accel.z /= caliber_value;

	imu_offset.gyro.x /= caliber_value;
	imu_offset.gyro.y /= caliber_value;
	imu_offset.gyro.z /= caliber_value;

	imu_offset.euler.roll /= caliber_value;
	imu_offset.euler.pitch /= caliber_value;
	imu_offset.euler.yaw /= caliber_value;

	prev_AccelX = imu_offset.accel.x;
	prev_AccelY = imu_offset.accel.y;
	prev_AccelZ = imu_offset.accel.z;

	prev_GyroX = imu_offset.gyro.x;
	prev_GyroY = imu_offset.gyro.y;
	prev_GyroZ = imu_offset.gyro.z;

	prev_EulerRoll = imu_offset.euler.roll;
	prev_EulerPitch = imu_offset.euler.pitch;
	prev_EulerYaw = imu_offset.euler.yaw;
}

void init_IMU()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	bno055_assignI2C(&hi2c1);

	HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, BNO055_SYS_TRIGGER, I2C_MEMADD_SIZE_8BIT, 0x20, 1, 1000);
	HAL_Delay(900);
	HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, BNO055_SYS_TRIGGER, I2C_MEMADD_SIZE_8BIT, 0x00, 1, 1000);
	bno055_setOperationModeConfig();
	bno055_setOperationModeNDOF();
	HAL_Delay(20);
	//    offset_IMU();
}

void read_IMU()
{
	bno055_vector_t accel = bno055_getVectorAccelerometer();
	bno055_vector_t gyro = bno055_getVectorGyroscope();
	bno055_vector_t euler = bno055_getVectorEuler();

	imu.accel.x = accel.x - imu_offset.accel.x;
	imu.accel.y = accel.y - imu_offset.accel.y;
	imu.accel.z = accel.z - imu_offset.accel.z;

	imu.gyro.x = gyro.x - imu_offset.gyro.x;
	imu.gyro.y = gyro.y - imu_offset.gyro.y;
	imu.gyro.z = gyro.z - imu_offset.gyro.z;

	imu.euler.roll = euler.x - imu_offset.euler.roll;
	imu.euler.pitch = euler.y - imu_offset.euler.pitch;
	imu.euler.yaw = euler.z - imu_offset.euler.yaw;
	//---------------------------Data Filtering----------------------------------//
	//    imu.accel.x = kalmanFilter(imu.accel.x, 6, 4, 0.1, prev_AccelX);
	//    imu.accel.y = kalmanFilter(imu.accel.y, 6, 4, 0.1, prev_AccelY);
	//    imu.accel.z = kalmanFilter(imu.accel.z, 6, 4, 0.1, prev_AccelZ);
	//
	//    imu.gyro.x = kalmanFilter(imu.gyro.x, 6, 4, 0.1, prev_GyroX);
	//    imu.gyro.y = kalmanFilter(imu.gyro.y, 6, 4, 0.1, prev_GyroY);
	//    imu.gyro.z = kalmanFilter(imu.gyro.z, 6, 4, 0.1, prev_GyroZ);
	//
	//    imu.euler.roll = kalmanFilter(imu.euler.roll, 6, 4, 0.1, prev_EulerRoll);
	//    imu.euler.pitch = kalmanFilter(imu.euler.pitch, 6, 4, 0.1, prev_EulerPitch);
	//    imu.euler.yaw = kalmanFilter(imu.euler.yaw, 6, 4, 0.1, prev_EulerYaw);

	imu.accel.x = yrt_simpleKalman_updateEstimate(&accelXKalman, imu.accel.x);
	imu.accel.y = yrt_simpleKalman_updateEstimate(&accelYKalman, imu.accel.y);
	imu.accel.z = yrt_simpleKalman_updateEstimate(&accelZKalman, imu.accel.z);

	imu.gyro.x = yrt_simpleKalman_updateEstimate(&gyroXKalman, imu.gyro.x);
	imu.gyro.y = yrt_simpleKalman_updateEstimate(&gyroYKalman, imu.gyro.y);
	imu.gyro.z = yrt_simpleKalman_updateEstimate(&gyroZKalman, imu.gyro.z);

	imu.euler.roll = yrt_simpleKalman_updateEstimate(&eulerYawKalman, imu.euler.roll);
	imu.euler.pitch = yrt_simpleKalman_updateEstimate(&eulerPitchKalman, imu.euler.pitch);
	imu.euler.yaw = yrt_simpleKalman_updateEstimate(&eulerRollKalman, imu.euler.yaw);

	prev_AccelX = imu.accel.x;
	prev_AccelY = imu.accel.y;
	prev_AccelZ = imu.accel.z;

	prev_GyroX = imu.gyro.x;
	prev_GyroY = imu.gyro.y;
	prev_GyroZ = imu.gyro.z;

	prev_EulerRoll = imu.euler.roll;
	prev_EulerPitch = imu.euler.pitch;
	prev_EulerYaw = imu.euler.yaw; // 0 - 360

	imu.accel.resultant = sqrt(pow(imu.accel.x, 2) + pow(imu.accel.x, 2) + pow(imu.accel.x, 2));


	accel_resultant_int16 = (int16_t)imu.accel.resultant*10;



	check_stabilization_state();

	if(flight_state == CONTROLLED_FLIGHT){
		check_out_of_control();
	}

}

#define UNSTABLE_VALUE 35.00
#define OUT_OF_CONTROL_VALUE 70.00

void check_stabilization_state()
{
	if (imu.euler.yaw > UNSTABLE_VALUE || imu.euler.yaw < (-1)*UNSTABLE_VALUE || imu.euler.pitch > UNSTABLE_VALUE || imu.euler.pitch < (-1)*UNSTABLE_VALUE)
	{
		stabilization_flag = UNSTABLE;
	}
	else
	{
		stabilization_flag = STABLE;
	}
}

uint8_t prev_flight_state;

void check_out_of_control(){

	if (imu.euler.yaw > OUT_OF_CONTROL_VALUE || imu.euler.yaw < (-1)*OUT_OF_CONTROL_VALUE || imu.euler.pitch > OUT_OF_CONTROL_VALUE || imu.euler.pitch < (-1)*OUT_OF_CONTROL_VALUE)
	{
		stabilization_flag = CONTROL_LOST;
		prev_flight_state = flight_state;
		flight_state = LOST_OF_CONTROL_STAGE;
	}
	else if (imu.euler.yaw > UNSTABLE_VALUE || imu.euler.yaw < (-1)*UNSTABLE_VALUE || imu.euler.pitch > UNSTABLE_VALUE || imu.euler.pitch < (-1)*UNSTABLE_VALUE)
	{
		stabilization_flag = UNSTABLE;
	}
	else
	{
		stabilization_flag = STABLE;
		flight_state = prev_flight_state;
	}


}


//-----------DATA-PACKING-----------//
/*
void dataPacking()
{
	  // Data pack:
	  //  flight_state, altitude(lander), velocity, acceleration(3), euler(3), max_altitude(state by state), max_velocity, max_accel?, container_altitude, lander_gps, container_gps,crc

	float_to_u8 float_converter;
	u16_to_u8 u16_converter;

	// flight state
	data_pack_final[0] = flight_state;
	// stabilization flag
	data_pack_final[1] = imu.stabilization_flag;
	// Altitude
	float_converter.u32 = altitude.altitude;
	data_pack_final[2] = float_converter.u8[0];
	data_pack_final[3] = float_converter.u8[1];
	data_pack_final[4] = float_converter.u8[2];
	data_pack_final[5] = float_converter.u8[3];

	// Velocity
	float_converter.u32 = velocity.vertical_velocity;
	data_pack_final[6] = float_converter.u8[0];
	data_pack_final[7] = float_converter.u8[1];
	data_pack_final[8] = float_converter.u8[2];
	data_pack_final[9] = float_converter.u8[3];

	// Acceleration
	float_converter.u32 = imu.accel.x;
	data_pack_final[10] = float_converter.u8[0];
	data_pack_final[11] = float_converter.u8[1];
	data_pack_final[12] = float_converter.u8[2];
	data_pack_final[13] = float_converter.u8[3];

	float_converter.u32 = imu.accel.y;
	data_pack_final[14] = float_converter.u8[0];
	data_pack_final[15] = float_converter.u8[1];
	data_pack_final[16] = float_converter.u8[2];
	data_pack_final[17] = float_converter.u8[3];

	float_converter.u32 = imu.accel.z;
	data_pack_final[18] = float_converter.u8[0];
	data_pack_final[19] = float_converter.u8[1];
	data_pack_final[20] = float_converter.u8[2];
	data_pack_final[21] = float_converter.u8[3];

	// Euler Angles
	float_converter.u32 = imu.euler.yaw;
	data_pack_final[22] = float_converter.u8[0];
	data_pack_final[23] = float_converter.u8[1];
	data_pack_final[24] = float_converter.u8[2];
	data_pack_final[25] = float_converter.u8[3];

	float_converter.u32 = imu.euler.pitch;
	data_pack_final[26] = float_converter.u8[0];
	data_pack_final[27] = float_converter.u8[1];
	data_pack_final[28] = float_converter.u8[2];
	data_pack_final[29] = float_converter.u8[3];

	float_converter.u32 = imu.euler.roll;
	data_pack_final[30] = float_converter.u8[0];
	data_pack_final[31] = float_converter.u8[1];
	data_pack_final[32] = float_converter.u8[2];
	data_pack_final[33] = float_converter.u8[3];

	// max_altitude
	float_converter.u32 = altitude.max_altitude;
	data_pack_final[34] = float_converter.u8[0];
	data_pack_final[35] = float_converter.u8[1];
	data_pack_final[36] = float_converter.u8[2];
	data_pack_final[37] = float_converter.u8[3];

	// max_velocity,
	float_converter.u32 = velocity.max_velocity;
	data_pack_final[38] = float_converter.u8[0];
	data_pack_final[39] = float_converter.u8[1];
	data_pack_final[40] = float_converter.u8[2];
	data_pack_final[41] = float_converter.u8[3];

	data_pack_final[42] = '\n';
	//sprintf(telemetry_str_packet,"%",flight_state,imu.stabilzation_state,velocity.vertical_velocity,imu.accel.x,imu.accel.y,imu.accel.z,imu.euler.yaw,imu.euler.pitch,imu.euler.roll,altitude.max_altitude,velocity.max_velocity);
}*/
uint8_t velocity_changing_determiner()
{

	if (imu.accel.z > 0)
		return 1;

	else
		return 0;

	velocity.velocity_changing = velocity_changing_determiner();
}

//----------------------UART-SENDING-----------------------//
char buffer_to_LTC[200];
void send_to_LTC_via_UART(){
	/*
	 * 	tick
	 *  flight state
	 *  stabilization state
	 *  altitude
	 *  max altitude
	 *  pressure
	 *  base pressure
	 *  vertical velocity
	 *  pressure
	 *  yaw
	 *  pitch
	 *  roll
	 *  resultant accel
	 *  accel x
	 *  accel y
	 *  accel z
	 *  gyro x
	 *  gyro y
	 *  gyro z
	 *
	 */
//	sprintf(buffer_to_LTC,"%lu,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",time.current,flight_state,stabilization_state,altitude.altitude,altitude.pressure,velocity.vertical_velocity,imu.euler.yaw,imu.euler.pitch,imu.euler.roll,imu.accel.resultant,thrust,servo1_angle,servo2_angle,servo3_angle,servo4_angle);
//	HAL_UART_Transmit(&huart2, &buffer_to_LTC, strlen(buffer_to_LTC), 100);
//	osDelay(10);

	sprintf(buffer_to_LTC,"%d,%d,%d,%f,%f,%f,%f,",time.current,flight_state,stabilization_flag,altitude.altitude,altitude.pressure,velocity.vertical_velocity,imu.accel.z);
	HAL_UART_Transmit(&huart2, &buffer_to_LTC, strlen(buffer_to_LTC), 100);
	osDelay(40);

	/*,%d,%d,%d,%d,%d*/
	sprintf(buffer_to_LTC,"%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n",imu.euler.yaw,imu.euler.pitch,imu.euler.roll,thrust,servo1_angle,servo2_angle,servo3_angle,servo4_angle,velocity.max_velocity_rocket_mach,velocity.max_velocity_payload_mach,altitude.temperature_int,altitude.pressure_int_2,accel_resultant_int16);
	HAL_UART_Transmit(&huart2, &buffer_to_LTC, strlen(buffer_to_LTC), 100);
	osDelay(100);

//	  char a[] = "Hello";
//	HAL_UART_Transmit(&huart2, &a, strlen(a), 100);
}

char buffer_to_LTC_test_version[200];

void send_to_LTC_via_UART_test_version(){
	/*
	 * 	tick
	 *  flight state
	 *  stabilization state
	 *  altitude
	 *  max altitude
	 *  pressure
	 *  base pressure
	 *  vertical velocity
	 *  pressure
	 *  yaw
	 *  pitch
	 *  roll
	 *  resultant accel
	 *  accel x
	 *  accel y
	 *  accel z
	 *  gyro x
	 *  gyro y
	 *  gyro z
	 *
	 */
	/*
	 * sprintf(buffer_to_LTC_test_version,"%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",time.current,flight_state,stabilization_state,altitude.max_altitude,altitude.altitude,altitude.pressure,altitude.base_pressure,velocity.vertical_velocity,imu.euler.yaw,imu.euler.pitch,imu.euler.roll,imu.accel.resultant,imu.accel.x,imu.accel.y,imu.accel.z,imu.gyro.x,imu.gyro.y,imu.gyro.z);
	 * HAL_UART_Transmit(&huart2, &buffer_to_LTC_test_version, strlen(buffer_to_LTC_test_version), 100);
	*/


	sprintf(buffer_to_LTC,"%lu,%d,%d,%f,%f,%f,%f,",time.current,flight_state,stabilization_flag,altitude.altitude,altitude.pressure,velocity.vertical_velocity,imu.accel.resultant);
	HAL_UART_Transmit(&huart2, &buffer_to_LTC, strlen(buffer_to_LTC), 100);
	osDelay(40);

	sprintf(buffer_to_LTC,"%f,%f,%f,%f,%f,%f,%f,%f\n",imu.euler.yaw,imu.euler.pitch,imu.euler.roll,thrust,servo1_angle,servo2_angle,servo3_angle,servo4_angle);
	HAL_UART_Transmit(&huart2, &buffer_to_LTC, strlen(buffer_to_LTC), 100);
	osDelay(100);

}
//----------------------------FOR-TEST-----------------------------------//
char rx_byte;
char rx_buffer[20];
int rx_buffer_counter = 0;
int rx_data;

HAL_StatusTypeDef status;

void emergency_shutdown_UART_init()
{
	status = HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void emergency_shutdown_UART()
{

	//	DC_motor_shutdown();
	if (rx_byte == '9')
	{

		system_emergency_shutdown();
	}

	else if (rx_byte == '1')
	{

		DC_motor_shutdown();
	}

	else if (rx_byte == '2')
	{
		thrust = 10;
	}

	else if (rx_byte == '3')
	{
		thrust = 20;
	}

	else if (rx_byte == '4')
	{
		thrust = 30;
	}

	else if (rx_byte == '5')
	{
		thrust = 50;
	}

	else if (rx_byte == '6')
	{
		thrust = 75;
	}

	else if (rx_byte == '7')
	{
		thrust = 100;
	}

	status = HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}
//---------------------------------------------------------------//
void getTime()
{
	time.current = HAL_GetTick();
	time.flight_time = time.current - time.flight_time_offset;
}

//----------------------------SD-CARD----------------------------//
char buffer_to_SD[530];

void SD_init()
{

	Mount_SD("/");

	Create_File("DCC_Data.txt");

	Unmount_SD("/");

	Mount_SD("/");

	getTime();
	// sprintf(buffer_to_SD,"Opening time: %lu",time.current);

	Update_File("DCC_Data.txt", buffer_to_SD);

	Unmount_SD("/");
}

// FRESULT sd_result;

void SD_write()
{
	Mount_SD("/");
	// sprintf(buffer_to_SD,"Tick: %lu,Flight_State: %d,Stabilization_Flag: %d,Altitude: %f,Max_Altitude: %f,Pressure: %f,Base_Pressure:%f,Velocity: %f,Yaw: %f,Pitch: %f,Roll: %f,AccelX: %f,AccelY: %f,AccelZ: %f,Abs_Accel: %f,GyroX: %f,GyroY: %f,GyroZ: %f,Thrust: %f\n",time.current,flight_state,stabilization_flag,altitude.altitude,altitude.max_altitude,altitude.pressure,altitude.base_pressure,velocity.vertical_velocity, imu.euler.yaw,imu.euler.pitch,imu.euler.roll,imu.accel.resultant,imu.accel.x,imu.accel.y,imu.accel.z,imu.gyro.x,imu.gyro.y,imu.gyro.z,thrust);
	sprintf(buffer_to_SD, "Tick: %lu,Flight_State: %d,Stabilization_Flag: %d,Altitude: %f, Temperature: %f,Pressure: %f,"
			"Base_Pressure:%f,Velocity: %f,Max_Rocket_Velocity: %f,Max_Payload_Velocity: %f,Yaw: %f,Pitch: %f,Roll: %f,"
			"AccelX: %f,AccelY: %f,AccelZ: %f,Abs_Accel: %f,GyroX: %f,GyroY: %f,GyroZ: %f,Thrust: %f, Servo1: %f,Servo2: %f,"
			"Servo3: %f,Servo4: %f,Flight_time: %d\n", time.current, flight_state, stabilization_flag, altitude.altitude, altitude.temperature,
			altitude.pressure, altitude.base_pressure, velocity.vertical_velocity, velocity.max_velocity_rocket, velocity.max_velocity_payload,
			imu.euler.yaw, imu.euler.pitch, imu.euler.roll, imu.accel.resultant, imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x, imu.gyro.y, imu.gyro.z,
			thrust, servo1_angle, servo2_angle, servo3_angle, servo4_angle,time.flight_time);

	Update_File("DCC_Data.txt", buffer_to_SD);

	Unmount_SD("/");
}

void give_recov_signal()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	osDelay(20000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void after_Landing(){
	if(altitude.altitude > 10 && flight_state == TOUCHDOWN){
		DC_motor_shutdown();
		servo_reset_state();
	}
}


void led1(uint8_t status)
{
	switch (status)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	case 1:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}

void led2(uint8_t status)
{
	switch (status)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	}
}

void led3(uint8_t status)
{
	switch (status)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	}
}

void system_check()
{/*
	if (sd_result == HAL_OK)
	{
		led1(1);
		osDelay(200);
		led1(0);
		osDelay(200);
	}*/

	if (altitude.altitude != 0)
	{
		led2(1);
		osDelay(200);
		led2(0);
		osDelay(200);
	}
	if (imu.euler.yaw != 0 && imu.euler.pitch != 0)
	{
		led3(1);
		osDelay(200);
		led3(0);
		osDelay(200);
	}
}
