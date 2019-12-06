
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
#include "STMGood.h"
#include "km_handle.h"
#include "chassis_thread.h"

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
			get_chassis_mode();

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
        kernal_ctrl.global_mode = SAFETY_MODE;
      }break;				
      case RC_MI:
      {
        kernal_ctrl.global_mode = MANUAL_CTRL_MODE;
      }break;		  		
      case RC_UP:
      {
        kernal_ctrl.global_mode = SEMI_AUTO_MODE;
      }break;				
		}
}

static void get_chassis_mode(void)
{	
    switch (kernal_ctrl.global_mode)  
		{
      case SAFETY_MODE:
      {
					chassis.mode = CHASSIS_RELAX;
      }break;				
      case MANUAL_CTRL_MODE:
      {
					chassis.mode = MANUAL_SEPARATE_GIMBAL;
      }break;		
      case SEMI_AUTO_MODE:
      {
					chassis.mode = UPSTAIR_MODE;
      }break;				
      case AUTO_CTRL_MODE:
      {

      }break;					
		}
}
static void chassis_mode_handle(void)
{
  switch (chassis.mode)
	{
      case CHASSIS_RELAX:
      {

      }break;					
      case CHASSIS_STOP:
      {

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
					chassis.UpStairVx = 3000;
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

/*  Shooting Revelent */

static void fric_speed_ctrl(void)
{
/*
	 swtich(FricSpeed)
	{
//   目标为敌方机器人单位
		case ROBOTATTACK_MODE:
		{
			
		}break;
//   目标为敌方能量机关
		case FANATTACK_MODE :
		{
			
		}break;	
//   目标为敌方 基地/前哨站
		case BASEATTACK_MODE :
		{
			
		}break;	 
// 无目标
		case NULL_MODE :
		{
			
		}break;		
	}
	
*/	
	
}

static void stir_freq_ctrl(void)
{
	
	
}





