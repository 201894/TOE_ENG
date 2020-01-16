
/** @file gimbal_task.c
 *  @version 1.0
 *  @date Dec 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#include "gimbal_thread.h"
#include "detect_thread.h"
#include "kernal_thread.h"
#include "chassis_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "STMGood.h"
#include "pid.h"
#include "user_lib.h"
#include "fetch_ctrl.h"

/* the * ratio of pit and yaw when using RC */
#define RC_YAW_RATIO             0.00063 // 0.0001    
#define RC_PIT_RATIO             0.002 //  0.0008 
/* the * ratio of pit and yaw when using KM */
#define KM_YAW_RATIO             0.0018 // 0.0001    
#define KM_PIT_RATIO             0.01 //  0.0008 
/* the limit of moto pit up/down */
#define PIT_MIN                  6200.0f   //    8.7   6200
#define PIT_MAX                  7160.0f  //    8.7   7160
/* the limit of moto yaw left/right*/
#define YAW_MIN                  2000.0f   //    8.7   6200
#define YAW_MAX                  4000.0f  //    8.7   7160
/* the limit of moto slip left/right */
#define SLIP_MIN                  -250.0f   //    8.7   6200
#define SLIP_MAX                  250.0f  //    8.7   7160
/* the limit of moto yaw left/right*/
#define FLIP_MIN                  -185.0f   //    
#define FLIP_MAX                  0.0f  //    

/*翻转 电机 PID 参数*/
#define  STIR_MAX_SPD                  	400 // 
#define  STIR_ANG_ERRORLLIM     	3000 // 
#define  STIR_MAX_CURRENT        	15000 // 
#define  STIR_SPD_ERRORLLIM       6000 // 
#define  STIR_ANG_KP                    	31.0f // 
#define  STIR_SPD_KP                    	42.0f // 
#define  STIR_SPD_KI                      	0.35 // 
#define  STIR_SPD_KD                    	0.0f // 
/* 摩擦轮电机 PID 参数*/
#define  FRIC_MAX_CURRENT        		15000 // 
#define  FRIC_SPD_ERRORLLIM       	6000 // 
#define  FRIC_SPD_KP           20.0f // 
#define  FRIC_SPD_KI            0.0f // 
#define  FRIC_SPD_KD           0.0f // 
/*滑移 电机 PID 参数*/
#define  SLIP_MAX_SPD                  	250 // 
#define  SLIP_ANG_ERRORLLIM     	3000 // 
#define  SLIP_MAX_CURRENT        		15000 // 
#define  SLIP_SPD_ERRORLLIM       	6000 // 
#define  SLIP_ANG_KP                    	30.0f // 
#define  SLIP_SPD_KP                    		60.0f // 
#define  SLIP_SPD_KI                      	0.35f // 
#define  SLIP_SPD_KD                    		0.0f // 
/*翻转 电机 PID 参数*/
#define  FLIP_MAX_SPD                  	400 // 
#define  FLIP_ANG_ERRORLLIM     	3000 // 
#define  FLIP_MAX_CURRENT        		15000 // 
#define  FLIP_SPD_ERRORLLIM       	6000 // 
#define  FLIP_ANG_KP                    	31.0f // 
#define  FLIP_SPD_KP                    		42.0f // 
#define  FLIP_SPD_KI                      	0.35 // 
#define  FLIP_SPD_KD                    		0.0f // 

gimbal_t gimbal;

void gimbal_pid_init(void)
{
	
  PID_struct_init(&pid_out[FlipECD], 0, FLIP_MAX_SPD, FLIP_ANG_KP, 0, 0); 
  PID_struct_init(&pid_out[SlipECD], 0, SLIP_MAX_SPD, SLIP_ANG_KP, 0, 0);	
	
  PID_struct_init(&pid_in[MotoLFlip],   FLIP_SPD_ERRORLLIM, FLIP_MAX_CURRENT, FLIP_SPD_KP, FLIP_SPD_KI, FLIP_SPD_KD);
  PID_struct_init(&pid_in[MotoRFlip],   FLIP_SPD_ERRORLLIM, FLIP_MAX_CURRENT, FLIP_SPD_KP, FLIP_SPD_KI, FLIP_SPD_KD);
  PID_struct_init(&pid_in[MotoSlip],     SLIP_SPD_ERRORLLIM, SLIP_MAX_CURRENT, SLIP_SPD_KP, SLIP_SPD_KI, SLIP_SPD_KD); 		
	
			/*  PID PARAMETER OF TWO 3508 FRIC MOTOR      */	
	PID_struct_init(&pid_fric[FricLeft],0,0,0,0,0);  
	PID_struct_init(&pid_fric[FricRight],0,0,0,0,0); 	
			/*  PID PARAMETER OF YAW MOTOR USING RAW ECD && RAW RPM      */
	PID_struct_init(&pid_out[MotoStir],0,0,0,0,0);  
	PID_struct_init(&pid_in[StirECD],0,0,0,0,0); 
			/*  外环    PID PARAMETER OF YAW MOTOR USING GYRO/RAW ECD      */
	PID_struct_init(&pid_out[YawIMU],0,0,0,0,0); 
			/*  外环    PID PARAMETER OF YAW MOTOR USING VISION      */
	PID_struct_init(&pid_out[YawVIS],0,0,0,0,0); 	
			/*  内环    PID PARAMETER OF YAW MOTOR USING GYRO/RAW RPM      */	
	PID_struct_init(&pid_in[MotoYaw],0,0,0,0,0);
			/*  外环    PID PARAMETER OF PIT MOTOR USING GYRO/RAW ECD      */
	PID_struct_init(&pid_out[PitECD],0,0,0,0,0);
			/*  外环    PID PARAMETER OF PIT MOTOR USING VISION      */
	PID_struct_init(&pid_out[PitVIS],0,0,0,0,0); 
			/*  内环    PID PARAMETER OF PIT MOTOR USING GYRO/RAW RPM      */		
	PID_struct_init(&pid_in[MotoPit],0,0,0,0,0);
	 
}

void gimbal_thread(void const * argument)
{
  /* USER CODE BEGIN logic_handle_task */
	portTickType GimbalHandleLastWakeTime;
  /* Infinite loop */
  for(;;)
  {
		GimbalHandleLastWakeTime = xTaskGetTickCount();		
		taskENTER_CRITICAL();

		gimbal_target_handle(&rc,&gimbal);	
//		gimbal_mode_switch();
		gimbal_pid_adjust();
		gimbal_pid_handle();		
		taskEXIT_CRITICAL();
    osDelayUntil(&GimbalHandleLastWakeTime,GIMBAL_THREAD_PERIOD);			
  }
  /* USER CODE END pid_handle_task */
}

static void gimbal_pid_adjust(void)
{
	#if 0
	    pid_adjust(&pid_out[FlipECD],_kp,0,0);
		  pid_out[FlipECD].MaxOut = maxout1;
	    pid_adjust(&pid_in[MotoLFlip],_kkp,_kki,_kkd);
	    pid_adjust(&pid_in[MotoRFlip],_kkp,_kki,_kkd);							
	#endif
	if(flip_to_min(30.0f)||flip_to_max(40.0f)){
		  pid_out[FlipECD].MaxOut = 300;	
	}
	else{
		  pid_out[FlipECD].MaxOut = 800;	
	}
}

static void gimbal_target_handle(rc_info_t *_rc , gimbal_t *_gimbal)
{	
   _gimbal->pitTargrtAngle += (float)(_rc->ch3 * RC_PIT_RATIO + _rc->mouse.x * KM_PIT_RATIO);
	 _gimbal->pitTargrtAngle = float_constrain(_gimbal->pitTargrtAngle,PIT_MIN,PIT_MAX);
   _gimbal->yawTargrtAngle += (float)(_rc->ch2 * RC_YAW_RATIO + _rc->mouse.y * KM_YAW_RATIO);	
//	 _gimbal->YawTargrtAngle = float_constrain(_gimbal->YawTargrtAngle,YAW_MIN,YAW_MAX); 
}

static void gimbal_mode_switch(void)
{
  /*  MODE BETWEEN IMU & VISION  */
  
}

static void gimbal_target_constrain(void)
{
	kernal_ctrl.flipTargetAngle = float_constrain(kernal_ctrl.flipTargetAngle,-185,0);
	kernal_ctrl.slipTargetPos = float_constrain(kernal_ctrl.slipTargetPos,-290,290);

}
	
static void gimbal_pid_handle(void)
{
		gimbal_target_constrain();
		/* 翻转电机PID控制    外环   理论目标位置           --        电机反馈位置  */
		pid_ast(&pid_out[FlipECD],kernal_ctrl.flipTargetAngle,moto_gimbal[MotoLFlip].total_angle);
		/* 翻转电机PID控制    内环   外环输出量             --        电机反馈速度  */		
		pid_ast(&pid_in[MotoLFlip],pid_out[FlipECD].ctrOut,moto_gimbal[MotoLFlip].speed_rpm * SPD_RATIO); 
		pid_ast(&pid_in[MotoRFlip],-pid_out[FlipECD].ctrOut,moto_gimbal[MotoRFlip].speed_rpm * SPD_RATIO);
		/* 滑移电机PID控制    外环   理论目标位置           --        电机反馈位置  */
		pid_ast(&pid_out[SlipECD],kernal_ctrl.slipTargetPos,moto_gimbal[MotoSlip].total_angle); 
		/* 滑移电机PID控制    内环   外环输出量             --        电机反馈位置  */	
		pid_ast(&pid_in[MotoSlip],pid_out[SlipECD].ctrOut,moto_gimbal[MotoSlip].speed_rpm * SPD_RATIO); 			
		                            /* 拨盘电机目标角度          电机反馈角度（初始值为 0）   */		
    pid_ast(&pid_out[StirECD], 0, moto_gimbal[MotoStir].total_angle);
		                            /* 拨盘外环输出量         电机反馈角速度   */				
    pid_ast(&pid_in[MotoStir],pid_out[MotoStir].ctrOut, moto_gimbal[MotoStir].speed_rpm* SPD_RATIO);
		if(chassis.canSendFlag == SET)
		{
	                              /* 发送计算得到的电流值 */
			send_gimbal_cur(0x200,(int16_t)pid_in[MotoLFlip].ctrOut,(int16_t)pid_in[MotoRFlip].ctrOut,(int16_t)pid_in[MotoSlip].ctrOut,(int16_t)pid_in[MotoStir].ctrOut);				
//		send_gimbal_cur(0x200,(int16_t)0,(int16_t)0,(int16_t)pid_in[MotoSlip].ctrOut,(int16_t)pid_in[MotoStir].ctrOut);				
		}
		else
		{
			send_gimbal_cur(0x200,(int16_t)0,(int16_t)0,(int16_t)0,(int16_t)0);						
		}
																	/* 摩擦轮目标转速            电机反馈角速度     */
    pid_ast(&pid_fric[FricLeft], kernal_ctrl.fricSpeed,moto_fric[FricLeft].speed_rpm);
    pid_ast(&pid_fric[FricRight],-kernal_ctrl.fricSpeed,moto_fric[FricRight].speed_rpm);		
		                            /* PIT轴 机械外环 + 视觉外环                                        电机反馈角速度*/
    pid_ast(&pid_in[MotoPit],(pid_out[PitECD].ctrOut + pid_out[PitVIS].ctrOut), moto_gimbal[MotoPit].speed_rpm);	
		                            /* YAW轴 机械外环 + 视觉外环                                        电机反馈角速度*/		
    pid_ast(&pid_in[MotoYaw],(pid_out[YawIMU].ctrOut + pid_out[YawVIS].ctrOut), moto_gimbal[MotoYaw].speed_rpm);
																/* 发送计算得到的电流值 */		
		send_gimbal_cur(0x1ff,(int16_t)pid_in[MotoPit].ctrOut,(int16_t)pid_in[MotoYaw].ctrOut,(int16_t)pid_fric[FricLeft].ctrOut,(int16_t)pid_fric[FricRight].ctrOut);		    
}




