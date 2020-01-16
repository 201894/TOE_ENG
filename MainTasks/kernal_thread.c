
/** @file __KERNAL_THREAD_H__
 *  @version 1.0
 *  @date  2019.5.12
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */

#include "kernal_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_io.h"
#include "pid.h"
#include "math.h"
#include "fetch_ctrl.h"
#include "STMGood.h"
#include "km_handle.h"
#include "chassis_thread.h"
#include "gimbal_thread.h"

kernal_ctrl_t kernal_ctrl;

void kernal_thread(void const * argument)
{
  /* USER CODE BEGIN KERNAL_THREAD */
	portTickType KernalHandleLastWakeTime;
  /* Infinite loop */
  for(;;)
  {
		KernalHandleLastWakeTime = xTaskGetTickCount();		
		taskENTER_CRITICAL();
		
			get_main_ctrl_mode();
			global_mode_handle();
				
			chassis_mode_handle();	
		
		taskEXIT_CRITICAL();
    osDelayUntil(&KernalHandleLastWakeTime,KERNAL_THREAD_PERIOD);			
  }
  /* USER CODE END KERNAL_THREAD */
}

/* Mode Switch Revelent */		

static void get_main_ctrl_mode(void)
{	
    switch (rc.sw1)  
		{
      case RC_DN:
      {
				switch (rc.sw2)  
				{
					case RC_DN:
					{
						kernal_ctrl.global_mode =  SAFETY_MODE;							
					}break;		
					case RC_MI:
					{
						kernal_ctrl.global_mode =  MANUAL_CTRL_MODE;		
					}break;		
					case RC_UP:
					{
						kernal_ctrl.global_mode =  SEMI_AUTO_MODE;								
					}break;	
			    default:{			
						kernal_ctrl.global_mode =  SAFETY_MODE;								
			    }
			    break;							
				}												
      }break;				
      case RC_MI:
      {
				switch (rc.sw2)  
				{
					case RC_DN:
					{
						kernal_ctrl.global_mode =  SEMI_AUTO_MODE;							
					}break;		
					case RC_MI: 
					{
						kernal_ctrl.global_mode =  SEMI_AUTO_MODE;		
					}break;	
					case RC_UP: 
					{
						kernal_ctrl.global_mode =  AUTO_FETCH_MODE1;										
					}break;	
			    default:{			
						kernal_ctrl.global_mode =  SAFETY_MODE;								
			    }
			    break;							
				}					
      }break;		  		
      case RC_UP:
      {
				switch (rc.sw2)  
				{
					case RC_DN:
					{
						kernal_ctrl.global_mode =  SEMI_AUTO_MODE;							
					}break;		
					case RC_MI: 
					{
						kernal_ctrl.global_mode =  AUTO_FETCH_MODE2;		
					}break;	
					case RC_UP: 
					{
						kernal_ctrl.global_mode =  SEMI_AUTO_MODE;								
					}break;	
			    default:{			
						kernal_ctrl.global_mode =  SAFETY_MODE;								
			    }
			    break;							
				}		
      }break;				
		}
}

static void global_mode_init(void)
{
		kernal_ctrl.fetchMode = 0;
		
}
static void global_mode_handle(void)
{	
	/*全局模式状态初始化更新*/
		global_mode_init();
	
    switch (kernal_ctrl.global_mode)  
		{
      case SAFETY_MODE:
      {
					chassis.mode = CHASSIS_RELAX;

      }break;				
      case MANUAL_CTRL_MODE:
      {
					chassis.mode = MANUAL_FOLLOW_GIMBAL;
      }break;		
      case SEMI_AUTO_MODE:
      {
					chassis.mode = MANUAL_SEPARATE_GIMBAL;					
      }break;				
      case AUTO_CTRL_MODE:
      {
					
      }break;		
      case AUTO_FETCH_MODE1:
      {
					chassis.mode = CHASSIS_STOP;  //  取弹模式 	
					kernal_ctrl.fetchMode = 1;				
      }break;	
      case AUTO_FETCH_MODE2:
      {
					chassis.mode = CHASSIS_STOP;  //  取弹模式 					
					kernal_ctrl.fetchMode = 2;
      }break;					
		}
}

static void chassis_mode_handle(void)
{
	
	chassis_state_init(); // 底盘状态 初始化 
	
  switch (chassis.mode)
	{
      case CHASSIS_RELAX:
      {
					chassis.canSendFlag = RESET;  // CAN2 pid处理电流值发送标志位置 0
      }break;					
      case CHASSIS_STOP:   
      {	
				UpLiftPos_Ctrl();
				if(uplift_accuracy(100.0f) == 1){
					chassis.stopFlag  = SET; 				
				}
					kernal_ctrl.flipTargetAngle -= (float)(rc.ch3)*0.002;
      }break;		
      case MANUAL_SEPARATE_GIMBAL:
      {
				chassis.targetPosition = LIFT_FETCH_ANGLE;  				
      }break;		
      case MANUAL_FOLLOW_GIMBAL:
      {
					
      }break;		
      case DODGE_MODE:
      {

      }break;		
      case AUTO_SEPARATE_GIMBAL:
      {

      }break;		
      case AUTO_FOLLOW_GIMBAL:
      {

      }break;		
      case UPSTAIR_MODE:
      {
					chassis.upStairVx = 3000;
      }break;					
	}
}

static void kb_enable_hook(void)
{
		if (rc.sw1 == RC_MI && rc.sw2 == RC_UP)
			km.kb_enable = 1;
		else
			km.kb_enable = 0;
}

void PID_InitArgument(void)
{
		chassis_pid_init();
		gimbal_pid_init();
}



