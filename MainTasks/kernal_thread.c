
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
						kernal_ctrl.global_mode =  RC_FETCH_MODE;								
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
						kernal_ctrl.global_mode =  MANUAL_CTRL_MODE;							
					}break;		
					case RC_MI: 
					{
						kernal_ctrl.global_mode =  BU_CLEAR_MODE;		
					}break;	
					case RC_UP: 
					{
						kernal_ctrl.global_mode =  BU_TEST_MODE;										
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
						kernal_ctrl.global_mode =  KB_CTRL_MODE;							
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
//		kernal_ctrl.fetchMode = 0;
		
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
      case RC_FETCH_MODE:
      {
					chassis.mode = CHASSIS_STOP;  //  取弹模式 				
	//				rc_mode_ctrl(kernal_ctrl.fetchMode,rc.ch0,rc.ch1);		
					rc_mode_ctrl(rc.ch0,rc.ch1);		
      }break;
      case BU_CLEAR_MODE:
      {
					
      }break;
      case BU_TEST_MODE:
      {				
      }break;				
      case KB_CTRL_MODE:
      {
					
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
      }break;		
      case MANUAL_SEPARATE_GIMBAL:
      {
				
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
static void rc_mode_ctrl(float _chx,float _chy)
{
		if(_chx >= 330){
			kernal_ctrl.fetchMode = 2;
		}
		if(_chx <= -330){
			kernal_ctrl.fetchMode = 3;	
		}
		if(_chy >= 330){
			kernal_ctrl.fetchMode = 1;				
		}
		if(_chy <= -330){
			kernal_ctrl.fetchMode = 0;				
		}
			
}

void PID_InitArgument(void)
{
		chassis_pid_init();
		gimbal_pid_init();
}



