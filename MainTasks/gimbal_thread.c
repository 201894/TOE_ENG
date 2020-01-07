
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
#include "cmsis_os.h"
#include "bsp_can.h"
#include "STMGood.h"
#include "pid.h"
#include "user_lib.h"


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
/*reduction ratio of moto 2006 36/1*/
#define C610_RATIO      (36/1)
gimbal_t gimbal;

void gimbal_pid_init(void)
{
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
//		gimbal_pid_handle();		
		taskEXIT_CRITICAL();
    osDelayUntil(&GimbalHandleLastWakeTime,GIMBAL_THREAD_PERIOD);			
  }
  /* USER CODE END pid_handle_task */
}

static void gimbal_target_handle(rc_info_t *_rc , gimbal_t *_gimbal)
{
	
   _gimbal->PitTargrtAngle += (float)(_rc->ch3 * RC_PIT_RATIO + _rc->mouse.x * KM_PIT_RATIO);
	 _gimbal->PitTargrtAngle = float_constrain(_gimbal->PitTargrtAngle,PIT_MIN,PIT_MAX);
   _gimbal->YawTargrtAngle += (float)(_rc->ch2 * RC_YAW_RATIO + _rc->mouse.y * KM_YAW_RATIO);	
//	 _gimbal->YawTargrtAngle = float_constrain(_gimbal->YawTargrtAngle,YAW_MIN,YAW_MAX); 
		slove_ms_send(kernal_ctrl.global_mode,_gimbal->YawTargrtAngle); 
}

static void gimbal_mode_switch(void)
{
  /*  MODE BETWEEN IMU & VISION  */
  
}

static void gimbal_pid_handle(void)
{
		                            /* 摩擦轮目标转速            电机反馈角速度     */
    pid_ast(&pid_fric[FricLeft], gimbal.fric_speed ,moto_fric[FricLeft].speed_rpm);
    pid_ast(&pid_fric[FricRight], -gimbal.fric_speed ,moto_fric[FricRight].speed_rpm);		
		                            /* 拨盘电机目标角度          电机反馈角度（初始值为 0）   */		
    pid_ast(&pid_out[StirECD], 0, 
		 moto_gimbal[MotoStir].total_angle);
		                            /* 拨盘外环输出量         电机反馈角速度   */				
    pid_ast(&pid_in[MotoStir],pid_out[MotoStir].ctrOut, moto_gimbal[MotoStir].speed_rpm);
	                              /* 发送计算得到的电流值 */
		send_gimbal_cur(0x200,0,0,0,(int16_t)pid_in[MotoStir].ctrOut);				
		                            /* PIT轴 机械外环 + 视觉外环                                        电机反馈角速度*/
    pid_ast(&pid_in[MotoPit],(pid_out[PitECD].ctrOut + pid_out[PitVIS].ctrOut), moto_gimbal[MotoPit].speed_rpm);	
		                            /* YAW轴 机械外环 + 视觉外环                                        电机反馈角速度*/		
    pid_ast(&pid_in[MotoYaw],(pid_out[YawIMU].ctrOut + pid_out[YawVIS].ctrOut), moto_gimbal[MotoYaw].speed_rpm);
																/* 发送计算得到的电流值 */		
		send_gimbal_cur(0x1ff,(int16_t)pid_in[MotoPit].ctrOut,(int16_t)pid_in[MotoYaw].ctrOut,(int16_t)pid_fric[FricLeft].ctrOut,(int16_t)pid_fric[FricRight].ctrOut);		    
}

