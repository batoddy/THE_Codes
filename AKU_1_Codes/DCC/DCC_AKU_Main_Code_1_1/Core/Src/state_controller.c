#include "state_controller.h"
#include "DCC_Motor_Control.h"
#include "DCC_Functions.h"
#include "DCC_Variables.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

float ref[6] = {0, 0, 0, 0, 0, 0}; // Reference Matrix
float X[6];						   // State Matrix
float E[6];						   // Error Matrix
float U[4];						   // Input Matrix
float K[4][6] = {{150.2613, 0.500, 283.6089, 4.4173, 1.5412, 0.0239},
				 {150.2613, 0.500, -1.5330, -0.0239, 285.8531, 4.4173},
				 {150.2613, 0.500, -283.6089, -4.4173, -1.5412, -0.0239},
				 {150.2613, 0.500, 1.5330, 0.0239, -285.8531, -4.4173}}; // Gain Matrix

float ref_velocity = -3;

PID pid;

void update_measurements()
{
	// BNO'nun eksenlerinin kontrol edilmesi lazım
	X[0] = deg_2_rad(imu.gyro.z);
	X[1] = deg_2_rad(imu.euler.roll - 180);
	X[2] = deg_2_rad(imu.gyro.y);
	X[3] = deg_2_rad(imu.euler.pitch);
	X[4] = deg_2_rad(imu.gyro.x);
	X[5] = deg_2_rad(imu.euler.yaw);

	osDelay(1);
}

void update_input()
{
	update_measurements();
	int i;
	int j;

	// Error calculation
	for (i = 0; i < 6; i++)
	{
		E[i] = ref[i] - X[i];
	}

	// Feedback
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 6; j++)
		{
			U[i] += K[i][j] * E[j];
		}
	}

	 //Saturaiton of inputs
		for (i = 0; i < 4; i++)
		{
			if (U[i] > 0.40)
			{
				U[i] = 0.40;
			}
			else if (U[i] < -0.40)
			{
				U[i] = -0.40;
			}
		}

	// SERVOLARIN DOĞRU EŞLEŞTİRİLMESİNE BAKILMASI GEREKLİ - KADİR

	servo1_angle = rad_2_deg(U[0]);
	servo2_angle = rad_2_deg(U[1]);
	servo3_angle = rad_2_deg(U[2]);
	servo4_angle = rad_2_deg(U[3]);

	servo1_pwm(servo1_angle);
	servo2_pwm(servo2_angle);
	servo3_pwm(servo3_angle);
	servo4_pwm(servo4_angle);

	osDelay(30);
}

float rad_2_deg(float radian)
{
	return radian * 180 / 3.141592;
}

float deg_2_rad(float degree)
{
	return degree * 3.141592 / 180;
}

void pid_init(float _kp, float _ki, float _kd)
{
	pid.Kp = _kp;
	pid.Ki = _ki;
	pid.Kd = _kd;

	pid.outmax = 100;
	pid.outmin = 0;
}

void pid_update()
{
	pid.error = ref_velocity - velocity.vertical_velocity;
	pid.time = HAL_GetTick();

	pid.prop = pid.Kp * pid.error;
	pid.integral += (pid.Ki * pid.error) * (pid.time - pid.prevTime);
	pid.derivative = pid.Kd * (pid.error - pid.prevError) / (pid.time - pid.prevTime);

	pid.prevError = pid.error;
	pid.prevTime = pid.time;

	pid.output = pid.prop + pid.derivative + pid.integral;

	if (pid.output > pid.outmax)
	{
		pid.output = pid.outmax;
	}
	else if (pid.output < pid.outmin)
	{
		pid.output = pid.outmin;
	}

	thrust = (int)pid.output;
	DC_motor_thrust(thrust);
}
