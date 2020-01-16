
/** @file __FETCH_CTRL_C__
 *  @version 1.0
 *  @date  2020.1.12
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */
 
#include "kernal_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "pid.h"
#include "fetch_ctrl.h"
#include "math.h"
#include "STMGood.h"
#include "km_handle.h"
#include "chassis_thread.h"
#include "gimbal_thread.h"


 
void chassis_state_init(void)
{
	chassis.canSendFlag 	= SET; // CAN2 pid处理电流值发送标志位默认置 1
	chassis.stopFlag 		= RESET;	  // 底盘状态标志位 默认 为 0，取弹模式下为 1
	kernal_ctrl.upLiftPosFlag = UPLIFT_POS_STATE;
	if(UPLIFT_POS_STATE== RESET){
		kernal_ctrl.flipTargetAngle = FLIP_INIT_ANGLE;
		kernal_ctrl.slipTargetPos = SLIP_INIT_ANGLE;	
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
	else {
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
uint8_t flip_to_max(float error)
{
	if (fabs(MAX_R_ANGLE - moto_gimbal[MotoLFlip].total_angle)<=error){
		return 1; 
	}
	else{ 
		return 0;				
	}
}	

uint8_t flip_to_min(float error)
{
	if (fabs(MIN_R_ANGLE - moto_gimbal[MotoLFlip].total_angle)<=error){
		return 1; 
	}
	else{ 
		return 0;
	}
}	
/* 抬升电机 翻转电机 自锁处理*/ 
void UpLiftPos_Ctrl(void)
{
//	if(flip_to_max(0.5)==1){
//			kernal_ctrl.upLiftSelfLockFlag = 1;
//	}
//	if(flip_to_min(90)==1){
//			kernal_ctrl.upLiftSelfLockFlag = 0;		
//	}
	if(kernal_ctrl.upLiftSelfLockFlag){
			chassis.targetPosition = LIFT_MAX_ANGLE;  
	}
	else{
			chassis.targetPosition = LIFT_FETCH_ANGLE;  				
	}
}

/*CDS5516 舵机控制函数*/
void servo_postion_ctrl(uint8_t flag)
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



