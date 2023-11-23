#include "DCC_Variables.h"


#define PI 3.141592
extern IMU imu;
extern Altitude altitude;

typedef struct
{
    float Kp, Ki, Kd;
    float tau;

    float limMin, limMax;

    float T;

    float error;
    float prop, integral, derivative;
    float prevError, prevMeas;
    float time, prevTime;

    float output;
    float outmax;
    float outmin; 
}PID;



// State feedback controller for vector control
void update_measurements();
void update_input();
float rad_2_deg(float radian);
float deg_2_rad(float degree);

//PID controller for BLDC motors
void pid_init(float _kp, float _ki, float _kd);
void pid_update();
