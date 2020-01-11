
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
#include "STMGood.h"
#include "km_handle.h"
#include "chassis_thread.h"
#include "gimbal_thread.h"

kernal_ctrl_t kernal_ctrl;
#define  LIFT_MAX_ANGLE       460.0f
#define  LIFT_INIT_ANGLE        1.5f
#define  SLIP_INIT_ANGLE        0.0f
#define  FLIP_INIT_ANGLE        -0.0
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

static void chassis_state_init(void)
{
	chassis.canSendFlag 	= SET; // CAN2 pid处理电流值发送标志位默认置 1
	chassis.stopFlag 		= RESET;	  // 底盘状态标志位 默认 为 0，取弹模式下为 1
	kernal_ctrl.upLiftPosFlag = UPLIFT_POS_STATE;
	if(UPLIFT_POS_STATE== RESET){
		kernal_ctrl.flipAngle 		= FLIP_INIT_ANGLE;
		kernal_ctrl.slipPos 			= SLIP_INIT_ANGLE;	
	}
	if(SLIP_POS_STATE){
		chassis.targetPosition = LIFT_INIT_ANGLE;
	}
}

	/* 返回抬升电机位置状态 精度在误差范围内返回1 否则返回0 */
uint8_t uplift_accuracy(float error)
{
	if (fabs(LIFT_MAX_ANGLE - moto_chassis[MotoLeftUpLift].total_angle)<=error){
		return 1; 
	}
	else{ 
		return 0;				
	}
}	
	/* 返回 SLIP 电机位置状态 精度在误差范围内返回1 否则返回0 */
uint8_t slip_accuracy(float error)
{
	if (fabs(SLIP_INIT_ANGLE - moto_gimbal[MotoSlip].total_angle)<=error){
		return 1; 
	}
	else{ 
		return 0;				
	}
}	
	/* 返回 FLIP 机位置状态 精度在误差范围内返回1 否则返回0 */
uint8_t flip_accuracy(float error)
{
	if (fabs(kernal_ctrl.flipAngle - moto_gimbal[MotoLFlip].total_angle)<=error){
		return 1; 
	}
	else{ 
		return 0;				
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
				chassis.targetPosition = LIFT_MAX_ANGLE;  
				if(uplift_accuracy(50.0f) == 1){
					chassis.stopFlag  = SET; 				
				} 
      }break;		
      case MANUAL_SEPARATE_GIMBAL:
      {
				chassis.targetPosition = LIFT_MAX_ANGLE;  				
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

/*舵机控制函数*/
static void servo_postion_ctrl(uint8_t flag)
{
	uint8_t dataSum;
	uint8_t servoData[] = {0xFF,0xFF,0x01,0x07,0x03,0x1E,0,0,0,0,0xFF};
	if(flag == 1)
	{
			servoData[6] =  0x3FF & 0XFF;
			servoData[7] =  0x3FF >> 8;	
			servoData[8] =  0x2FF & 0XFF;
			servoData[9] =  0x2FF >> 8;			
	}
	else
	{
			servoData[6] =  0x00 & 0XFF;
			servoData[7] =  0x00 >> 8;			
			servoData[8] =  0x1FF & 0XFF;
			servoData[9] =  0x1FF >> 8;					
	}	
	
	for(int i = 2; i < 10; i++)
	{
			dataSum += servoData[i];
	}
	
	servoData[10] = ~dataSum;
	
	for(int i = 0; i  < 11; i++)
	{
		USART_SendChar(servoData[i],USART2);
	}
}

static void stir_freq_ctrl(void)
{
	
	
}

void PID_InitArgument(void)
{
		chassis_pid_init();
		gimbal_pid_init();
}



