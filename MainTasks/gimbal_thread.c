
/** @file gimbal_task.c
 *  @version 1.0
 *  @date Dec 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#include "gimbal_thread.h"
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

gimbal_t gimbal;

void gimbal_param_init(void)
{
			/*  PID PARAMETER OF TWO 3508 FRIC MOTOR      */	
	PID_struct_init(&pid_fric[FricLeft],0,0,0,0,0);  
	PID_struct_init(&pid_fric[FricRight],0,0,0,0,0); 	
			/*  PID PARAMETER OF YAW MOTOR USING RAW ECD && RAW RPM      */
	PID_struct_init(&pid_out[MotoStir],0,0,0,0,0);  
	PID_struct_init(&pid_in[MotoStir],0,0,0,0,0); 
			/*  �⻷    PID PARAMETER OF YAW MOTOR USING GYRO/RAW ECD      */
	PID_struct_init(&pid_out[YawNormal],0,0,0,0,0); 
			/*  �⻷    PID PARAMETER OF YAW MOTOR USING VISION      */
	PID_struct_init(&pid_out[YawVision],0,0,0,0,0); 	
			/*  �ڻ�    PID PARAMETER OF YAW MOTOR USING GYRO/RAW RPM      */	
	PID_struct_init(&pid_in[MotoYaw],0,0,0,0,0);
			/*  �⻷    PID PARAMETER OF PIT MOTOR USING GYRO/RAW ECD      */
	PID_struct_init(&pid_out[PitNormal],0,0,0,0,0);
			/*  �⻷    PID PARAMETER OF PIT MOTOR USING VISION      */
	PID_struct_init(&pid_out[PitVision],0,0,0,0,0); 
			/*  �ڻ�    PID PARAMETER OF PIT MOTOR USING GYRO/RAW RPM      */		
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
		gimbal_mode_switch();
		gimbal_pid_handle();
		
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
	 _gimbal->YawTargrtAngle = float_constrain(_gimbal->YawTargrtAngle,YAW_MIN,YAW_MAX); 
	
}

static void gimbal_mode_switch(void)
{
  /*  MODE BETWEEN IMU & VISION  */
  
}

static void gimbal_pid_handle(void)
{
		                            /* Ħ����Ŀ��ת��            ����������ٶ�     */
    pid_ast(&pid_fric[FricLeft], gimbal.fric_speed ,moto_fric[FricLeft].speed_rpm);
    pid_ast(&pid_fric[FricRight], -gimbal.fric_speed ,moto_fric[FricRight].speed_rpm);		
		                            /* ���̵��Ŀ��Ƕ�          ��������Ƕȣ���ʼֵΪ 0��   */		
    pid_ast(&pid_out[MotoStir], gimbal.fric_speed, 
		 moto_gimbal[MotoStir].total_angle - moto_gimbal[MotoStir].offset_angle);
		                            /* �����⻷�����         ����������ٶ�   */				
    pid_ast(&pid_in[StirNormal],pid_out[MotoStir].ctrOut, moto_gimbal[MotoStir].speed_rpm);
	                              /* ���ͼ���õ��ĵ���ֵ */
		send_gimbal_cur(0x200,(int16_t)pid_in[StirNormal].ctrOut,(int16_t)pid_fric[FricLeft].ctrOut,(int16_t)pid_fric[FricRight].ctrOut,0);				
		                            /* PIT�� ��е�⻷ + �Ӿ��⻷                                        ����������ٶ�*/
    pid_ast(&pid_in[MotoPit],(pid_out[PitNormal].ctrOut + pid_out[PitVision].ctrOut), moto_gimbal[MotoPit].speed_rpm);	
		                            /* YAW�� ��е�⻷ + �Ӿ��⻷                                        ����������ٶ�*/		
    pid_ast(&pid_in[MotoYaw],(pid_out[YawNormal].ctrOut + pid_out[YawVision].ctrOut), moto_gimbal[MotoYaw].speed_rpm);
																/* ���ͼ���õ��ĵ���ֵ */		
		send_gimbal_cur(0x1ff,(int16_t)pid_in[MotoPit].ctrOut,(int16_t)pid_in[MotoYaw].ctrOut,0,0);		    
}



