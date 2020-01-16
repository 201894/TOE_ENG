/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "bsp_io.h"
#include "km_handle.h"
#include "bsp_uart.h"
#include "detect_thread.h"
#include "gimbal_thread.h"
#include "bsp_can.h"
#include "chassis_thread.h"
#include "pid.h" 
#include "iwdg.h"
#include "kernal_thread.h"
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
osThreadId DEBUGTHREADHandle;
osThreadId CHASSISTHREADHandle;
osThreadId GIMBALTHREADHandle;
osThreadId KERNALTHREADHandle;
osThreadId DETECTHREADHandle;
osThreadId IMUTHREADHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void debug_thread(void const * argument);
void chassis_thread(void const * argument);
void gimbal_thread(void const * argument);
void kernal_thread(void const * argument);
void detect_thread(void const * argument);
void imu_thread(void const * argument);

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
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  /* definition and creation of DEBUGTHREAD */
  osThreadDef(DEBUGTHREAD, debug_thread, osPriorityIdle, 0, 256);
  DEBUGTHREADHandle = osThreadCreate(osThread(DEBUGTHREAD), NULL);

  /* definition and creation of CHASSISTHREAD */
  osThreadDef(CHASSISTHREAD, chassis_thread, osPriorityIdle, 0, 256);
  CHASSISTHREADHandle = osThreadCreate(osThread(CHASSISTHREAD), NULL);

  /* definition and creation of GIMBALTHREAD */
  osThreadDef(GIMBALTHREAD, gimbal_thread, osPriorityIdle, 0, 256);
  GIMBALTHREADHandle = osThreadCreate(osThread(GIMBALTHREAD), NULL);

  /* definition and creation of KERNALTHREAD */
  osThreadDef(KERNALTHREAD, kernal_thread, osPriorityNormal, 0, 256);
  KERNALTHREADHandle = osThreadCreate(osThread(KERNALTHREAD), NULL);

  /* definition and creation of DETECTHREAD */
  osThreadDef(DETECTHREAD, detect_thread, osPriorityIdle, 0, 128);
  DETECTHREADHandle = osThreadCreate(osThread(DETECTHREAD), NULL);

  /* definition and creation of IMUTHREAD */
  osThreadDef(IMUTHREAD, imu_thread, osPriorityIdle, 0, 128);
  IMUTHREADHandle = osThreadCreate(osThread(IMUTHREAD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_debug_thread */
/**
  * @brief  Function implementing the DEBUGTHREAD thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_debug_thread */
void debug_thread(void const * argument)
{
  /* USER CODE BEGIN debug_thread */
  /* Infinite loop */
  for(;;)
  {		
/*
			typedef enum
			{  
				MotoLeftUp = 0,MotoRightUp= 1, MotoLeftDown = 2,MotoRightDown= 3,MotoMidUp = 4,	 	MotoMidDown= 5,	MotoNumber= 6,
				MaxId= 20,		
			}ChassisModule_ID;	
			typedef enum
			{	
				 STIR,YAW,PIT,FRIC,CHASSIS,JUDGE,NUC,DEBUS,IMU,COMMU,GYROY,GYROP, 
			}module_fps_e;		
*/
#if 1
		printf("BULLET FLAG REVENLENT : ##\r\n");		
		printf("upLiftPosFlag = %d\r\n",kernal_ctrl.upLiftPosFlag);
		printf("stopFlag = %d\r\n",chassis.stopFlag);		
		printf("TargetLiftAngle = %.2f\r\n",chassis.targetPosition);	
		printf("CurrentAngle = %.2f\r\n",moto_gimbal[MotoLFlip].total_angle);			
		printf("	Target slipPos = %.2f\r\n",	kernal_ctrl.slipTargetPos);	
		printf("	Target.flipAngle = %.2f\r\n",kernal_ctrl.flipTargetAngle);			
		printf(".targetPosition = %.2f\r\n",chassis.targetPosition);			
		printf("SLIP_SWITCH = %d\r\n",Slip_SWITCH);			
#endif		
		
#if 0
		printf("CHASSIS REVENLENT : ##\r\n");		
		printf("chassis[LeftUp].speed = %d\r\n",moto_chassis[MotoLeftUp].speed_rpm);			
		printf("chassis[RightUp].speed = %d\r\n",moto_chassis[MotoRightUp].speed_rpm);				
		printf("chassis[LeftDown].speed = %d\r\n",moto_chassis[MotoLeftDown].speed_rpm);				
		printf("chassis[RightDown].speed = %d\r\n",moto_chassis[MotoRightDown].speed_rpm);				
		printf("chassis[MidUp].speed = %d\r\n",moto_chassis[MotoMidUp].speed_rpm);				
		printf("chassis[MidDown].speed = %d\r\n",moto_chassis[MotoMidDown].speed_rpm);				
#endif		
#if 1
		printf("##FPS REVENLENT : ##\r\n");		
//		printf("fps[STIR] = %d\r\n",r_fps[STIR].fps);
//		printf("fps[YAW] = %d\r\n",r_fps[YAW].fps);
//		printf("fps[PIT] = %d\r\n",r_fps[PIT].fps);		
//		printf("fps[FRIC] = %d\r\n",r_fps[FRIC].fps);		
//		printf("fps[CHASSIS] = %d\r\n",r_fps[CHASSIS].fps);    
//		printf("fps[JUDGE] = %d\r\n",r_fps[JUDGE].fps);			
//		printf("fps[NUC] = %d\r\n",r_fps[NUC].fps);					
		printf("fps[DEBUS] = %d\r\n",r_fps[DEBUS].fps);  
		printf("fps[LUP] = %d\r\n",r_fps[LUP].fps);  	 
		printf("fps[RUP] = %d\r\n",r_fps[RUP].fps);  	
		printf("fps[SLIP] = %d\r\n",r_fps[SLIP].fps);  
		printf("fps[RFLIP] = %d\r\n",r_fps[RFLIP].fps);  	 
		printf("fps[LFLIP] = %d\r\n",r_fps[LFLIP].fps);  			
//		printf("fps[IMU] = %d\r\n",r_fps[IMU].fps);  	
		printf("fps[COMMU] = %d\r\n",r_fps[COMMU].fps);			
//		printf("fps[GYROY] = %d\r\n",r_fps[GYROY].fps);			
//		printf("fps[GYROP] = %d\r\n",r_fps[GYROP].fps);					
#endif
		
#if 0
				printf("##RC REVENLENT : ##\r\n");		
				printf("rc.ch0 = %d\r\n",rc.ch0);
				printf("rc.ch1 = %d\r\n",rc.ch1);		
				printf("rc.sw1 = %d\r\n",rc.sw1);		
				printf("rc.sw2 = %d\r\n",rc.sw2);						
				printf("chassis,vx = %.3f\r\n",chassis.vx);		
				printf("YawTargrtAngle = %.3f\r\n",gimbal.YawTargrtAngle);						
#endif
#if 0
				printf("##KM REVENLENT : ##\r\n");		
				printf("rc.mouse.l = %d\r\n",rc.mouse.l);
				printf("rc.mouse.x = %d\r\n",rc.mouse.x);		
				printf("key_code = %d\r\n",rc.kb.key_code);		
#endif
		 
#if 0
			printf("# UPLIFT PID REVENLENT : #\r\n");				
			printf("TargetAngle = %.2f\r\n",chassis.targetPosition);	
			printf("Currentecd = %d\r\n",moto_chassis[MotoLeftUpLift].ecd);				
			printf("CurrentAngle = %.2f\r\n",moto_chassis[MotoLeftUpLift].total_angle);	
			printf("AngleError = #%.2f\r\n",pid_out[LiftECD].errNow);	
			printf("FirstCtrOut = %.2f\r\n",pid_out[LiftECD].ctrOut);				
			printf("CurrentSpd = %.2f\r\n",moto_chassis[MotoLeftUpLift].speed_rpm*0.2);	 // MotoData[RightUpLift].speed_rpm
			printf("CurrentSpdRight = %.2f\r\n",moto_chassis[MotoRightUpLift].speed_rpm*0.2);	 // MotoData[RightUpLift].speed_rpm			
			printf("SpdError = %.2f\r\n",pid_in[MotoLUpLft].errNow);			
			printf("FinalCtrOut = %d\r\n",(int16_t)pid_in[MotoLUpLft].ctrOut);					
			printf("RightFinalCtrOut = %d\r\n",(int16_t)pid_in[MotoRUpLft].ctrOut);					
#endif	

#if 0
			printf("# SLIP PID REVENLENT : #\r\n");				
			printf("	kernal_ctrl.slipPos = %d\r\n",	kernal_ctrl.slipTargetPos);	
			printf("Currentecd = %d\r\n",moto_gimbal[MotoSlip].ecd);				
			printf("CurrentAngle = %.2f\r\n",moto_gimbal[MotoSlip].total_angle);	
			printf("AngleError = #%.2f\r\n",pid_out[SlipECD].errNow);	
			printf("FirstCtrOut = %.2f\r\n",pid_out[SlipECD].ctrOut);				
			printf("CurrentSpd = %.2f\r\n",moto_gimbal[MotoSlip].speed_rpm*0.2);	 // MotoData[RightUpLift].speed_rpm
			printf("SpdError = %.2f\r\n",pid_in[MotoSlip].errNow);			
			printf("FinalCtrOut = %d\r\n",(int16_t)pid_in[MotoSlip].ctrOut);								
#endif	

#if 0
			printf("# FLIP PID REVENLENT : #\r\n");				
			printf("MaxOut = %.2f\r\n",pid_out[FlipECD].MaxOut);				
			printf("TargetAngle = %.2f\r\n",kernal_ctrl.flipTargetAngle);	
			printf("Currentecd = %d\r\n",moto_gimbal[MotoLFlip].ecd);				
			printf("CurrentAngle = %.2f\r\n",moto_gimbal[MotoLFlip].total_angle);	
			printf("AngleError = #%.2f\r\n",pid_out[FlipECD].errNow);	
			printf("FirstCtrOut = %.2f\r\n",pid_out[FlipECD].ctrOut);				
			printf("CurrentSpd = %.2f\r\n",moto_gimbal[MotoLFlip].speed_rpm*0.2);	 // MotoData[RightUpLift].speed_rpm
			printf("CurrentSpdRight = %.2f\r\n",moto_gimbal[MotoLFlip].speed_rpm*0.2);	 // MotoData[RightUpLift].speed_rpm			
			printf("SpdError = %.2f\r\n",pid_in[MotoLFlip].errNow);			
			printf("FinalCtrOut = %d\r\n",(int16_t)pid_in[MotoLFlip].ctrOut);					
			printf("RightFinalCtrOut = %d\r\n",(int16_t)pid_in[MotoRFlip].ctrOut);					
#endif	

    if (g_err.err_now == NULL)
    {
			LED_G_TOG;
		}
//			ledFlow(1600,8);

		  osDelay(200);
  }
  /* USER CODE END debug_thread */
}

/* USER CODE BEGIN Header_chassis_thread */
/**
* @brief Function implementing the CHASSISTHREAD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_thread */
__weak void chassis_thread(void const * argument)
{
  /* USER CODE BEGIN chassis_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_thread */
}

/* USER CODE BEGIN Header_gimbal_thread */
/**
* @brief Function implementing the GIMBALTHREAD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_thread */
__weak void gimbal_thread(void const * argument)
{
  /* USER CODE BEGIN gimbal_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_thread */
}

/* USER CODE BEGIN Header_kernal_thread */
/**
* @brief Function implementing the KERNALTHREAD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kernal_thread */
__weak void kernal_thread(void const * argument)
{
  /* USER CODE BEGIN kernal_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END kernal_thread */
}

/* USER CODE BEGIN Header_detect_thread */
/**
* @brief Function implementing the DETECTHREAD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_thread */
__weak void detect_thread(void const * argument)
{
  /* USER CODE BEGIN detect_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END detect_thread */
}

/* USER CODE BEGIN Header_imu_thread */
/**
* @brief Function implementing the IMUTHREAD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_thread */
__weak void imu_thread(void const * argument)
{
  /* USER CODE BEGIN imu_thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
