/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DCC_Functions.h"
#include "DCC_Variables.h"
#include "DCC_Motor_Control.h"
#include "DCC_Checking.h"
#include "tim.h"
#include "File_Handling_RTOS.h"
#include "state_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Read_SensorDataHandle;
osThreadId Flight_StatesHandle;
osThreadId Servo_ControlHandle;
osThreadId DC_Motor_ControHandle;
osThreadId Uart_to_LTCHandle;
osThreadId Write_to_SDHandle;
osThreadId System_CheckHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Read_Sensor_Data_Task(void const * argument);
void Flight_States_Task(void const * argument);
void Servo_Control_Task(void const * argument);
void DC_Motor_Control_Task(void const * argument);
void Uart_to_LTC_Task(void const * argument);
void Write_to_SD_Start(void const * argument);
void System_Check_Start(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Read_SensorData */
  osThreadDef(Read_SensorData, Read_Sensor_Data_Task, osPriorityRealtime, 0, 512);
  Read_SensorDataHandle = osThreadCreate(osThread(Read_SensorData), NULL);

  /* definition and creation of Flight_States */
  osThreadDef(Flight_States, Flight_States_Task, osPriorityHigh, 0, 512);
  Flight_StatesHandle = osThreadCreate(osThread(Flight_States), NULL);

  /* definition and creation of Servo_Control */
  osThreadDef(Servo_Control, Servo_Control_Task, osPriorityHigh, 0, 512);
  Servo_ControlHandle = osThreadCreate(osThread(Servo_Control), NULL);

  /* definition and creation of DC_Motor_Contro */
  osThreadDef(DC_Motor_Contro, DC_Motor_Control_Task, osPriorityHigh, 0, 512);
  DC_Motor_ControHandle = osThreadCreate(osThread(DC_Motor_Contro), NULL);

  /* definition and creation of Uart_to_LTC */
  osThreadDef(Uart_to_LTC, Uart_to_LTC_Task, osPriorityNormal, 0, 2048);
  Uart_to_LTCHandle = osThreadCreate(osThread(Uart_to_LTC), NULL);

  /* definition and creation of Write_to_SD */
  osThreadDef(Write_to_SD, Write_to_SD_Start, osPriorityNormal, 0, 1024);
  Write_to_SDHandle = osThreadCreate(osThread(Write_to_SD), NULL);

  /* definition and creation of System_Check */
  osThreadDef(System_Check, System_Check_Start, osPriorityHigh, 0, 512);
  System_CheckHandle = osThreadCreate(osThread(System_Check), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Read_Sensor_Data_Task */
/**
 * @brief  Function implementing the Read_SensorData thread.
 * @param  argument: Not used
 * @retval None
 */
int data_collecting_time;
int temp_time;
/* USER CODE END Header_Read_Sensor_Data_Task */
void Read_Sensor_Data_Task(void const * argument)
{
  /* USER CODE BEGIN Read_Sensor_Data_Task */
  initFilter();
  initBarometer();
  init_IMU();
  /* Infinite loop */
  for (;;)
  {
    temp_time = HAL_GetTick();
    read_IMU();
    readAltitude();
    getTime();

    data_collecting_time = HAL_GetTick() - temp_time;
    osDelay(1);
  }
  /* USER CODE END Read_Sensor_Data_Task */
}

/* USER CODE BEGIN Header_Flight_States_Task */
/**
 * @brief Function implementing the Flight_States thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Flight_States_Task */
void Flight_States_Task(void const * argument)
{
  /* USER CODE BEGIN Flight_States_Task */
  /* Infinite loop */
  for (;;)
  {
    switch (flight_state)
    {
    case START:

      if (checkLiftoff() == 1)
      {
    	// time.liftoff = time.current;
        flight_state = AFTER_LIFTOFF;
      }
      break;

    case AFTER_LIFTOFF:

      if (checkBurnout() == 1)
      {

        flight_state = AFTER_BURNOUT;
      }
      break;

    case AFTER_BURNOUT:

      if (checkApogee() == 1)
      {
    	// time.apogee = time.current;
        flight_state = FREEFALL_STAGE;
      }
      break;

      //	    case AFTER_APOGEE:
      //	      if (checkSeperationfromRocket() == 1)
      //	      {
      //	        flight_state = FREEFALL_STAGE;
      //	      }
      //      break;
    case FREEFALL_STAGE:

      if (checkSeperationfromContainer() == 1)
      {
        // time.container_seperation = time.current;
    	recovery_command = 1;
    	osDelay(3000);
        flight_state = CONTROLLED_FLIGHT;
      }
      break;

    case CONTROLLED_FLIGHT:

      if (checkLanding() == 1)
      {
    	// time.landing = time.current;
        flight_state = TOUCHDOWN;
      }
      break;

    case TOUCHDOWN:

    	osDelay(2000);
    	DC_motor_shutdown();
    	servo_reset_state();
      break;

    case LOST_OF_CONTROL_STAGE:

    	DC_motor_shutdown();
    	servo_reset_state();
    	break;

    default:
                    // DEBUG_PROFILE_X
    	break;
    }

    osDelay(1);
  }
  /* USER CODE END Flight_States_Task */
}

/* USER CODE BEGIN Header_Servo_Control_Task */
/**
 * @brief Function implementing the Servo_Control thread.
 * @param argument: Not used
 * @retval None
 */


/* USER CODE END Header_Servo_Control_Task */
void Servo_Control_Task(void const * argument)
{
  /* USER CODE BEGIN Servo_Control_Task */
  init_Servo();

  servo_movement_control();

  /* Infinite loop */
  for (;;)
  {

    if (flight_state >= CONTROLLED_FLIGHT)
    {
    	if(flight_state == TOUCHDOWN || flight_state == LOST_OF_CONTROL_STAGE){
    		osDelay(2000);
    		servo_reset_state();
    	}
    	else {
    		update_input();
    	}
    }
    else
    {
       servo_reset_state();
    }
    osDelay(1);
  }
  /* USER CODE END Servo_Control_Task */
}

/* USER CODE BEGIN Header_DC_Motor_Control_Task */
/**
 * @brief Function implementing the DC_Motor_Contro thread.
 * @param argument: Not used
 * @retval None
 */
int cntr;
/* USER CODE END Header_DC_Motor_Control_Task */
void DC_Motor_Control_Task(void const * argument)
{
  /* USER CODE BEGIN DC_Motor_Control_Task */
   init_DC_Motor();

   configurate_ESC();

   pid_init(0.8826413834408723,0.111547323582044,-0.281529513439454);

  /* Infinite loop */
  for (;;)
  {
	  //DC_motor_thrust(100);
	    if (flight_state >= CONTROLLED_FLIGHT)
	    {
	    	if(flight_state == TOUCHDOWN || flight_state == LOST_OF_CONTROL_STAGE){
	    		osDelay(2000);
	    		DC_motor_shutdown();
	    	}
	    	else {
	    		pid_update();
	    	}
	    }
	    else
	    {
	       servo_reset_state();
	    }
       osDelay(1);
  }
  /* USER CODE END DC_Motor_Control_Task */
}

/* USER CODE BEGIN Header_Uart_to_LTC_Task */
/**
 * @brief Function implementing the Uart_to_LTC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Uart_to_LTC_Task */
void Uart_to_LTC_Task(void const * argument)
{
  /* USER CODE BEGIN Uart_to_LTC_Task */
  // emergency_shutdown_UART_init();
  osDelay(8000);


  /* Infinite loop */
  for (;;)
  {

    send_to_LTC_via_UART();
    osDelay(20);
  }
  /* USER CODE END Uart_to_LTC_Task */
}

/* USER CODE BEGIN Header_Write_to_SD_Start */
/**
 * @brief Function implementing the Write_to_SD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Write_to_SD_Start */
void Write_to_SD_Start(void const * argument)
{
  /* USER CODE BEGIN Write_to_SD_Start */
  SD_init();
  /* Infinite loop */
  for (;;)
  {
    SD_write();

    osDelay(1);
  }
  /* USER CODE END Write_to_SD_Start */
}

/* USER CODE BEGIN Header_System_Check_Start */
/**
 * @brief Function implementing the System_Check thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_System_Check_Start */
void System_Check_Start(void const * argument)
{
  /* USER CODE BEGIN System_Check_Start */
  /* Infinite loop */
  for (;;)
  {

	  if(recovery_command == 1){
	      			  give_recov_signal();
	      			  recovery_command = 0;
	      	}
    osDelay(1);
  }
  /* USER CODE END System_Check_Start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
