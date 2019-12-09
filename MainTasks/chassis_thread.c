
/** @file __CHASSIS_THREAD_H__
 *  @version 1.1
 *  @date Dec 2019
 *
 *  @brief handle the chassis move and pid handle
 *
 */
 

#include "chassis_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "STMGood.h"
#include "pid.h"
#include "string.h"
#include "math.h"

#define CHASSIS_THREAD_PERIOD 	5
#define GIMBAL_POSITION_RATIO		 	2.0f
#define GIMBAL_MIDDLE_ANGLE   		100.0f
#define GIMBAL_MIDDLE_ECD     			4000
#define NORMAL_PARAM						12.12f
/*扭腰模式相关参数*/
/* 单位时间模式    扭腰周期 */
#define GORGI_PERIOD                       1600
/* 云台电机模式    扭腰幅度 */
#define GORGI_GM_ANGLE                 40.0f

#define GORGI_GM_OFFSET                8.0f
chassis_t chassis;

void chassis_pid_init(void)
{
	for(int i=0;i<6;i++)
	{
	  memset(&moto_chassis[i],0,sizeof(moto_param));
	  PID_struct_init(&pid_chassis[i],6000,15000,0,0,0);              //          
	} 
	
	PID_struct_init(&pid_out[CGLink],0,0,0,0,0);  
  PID_struct_init(&pid_in[LinkNormal],0,0,0,0,0);  
}

void chassis_thread(void const * argument)
{
  /* USER CODE BEGIN logic_handle_task */
	portTickType ChassisHandleLastWakeTime;
  /* Infinite loop */
  for(;;)
  {
			ChassisHandleLastWakeTime = xTaskGetTickCount();		
			taskENTER_CRITICAL();
			
				chassis_link_handle();
				for(int i = MotoLeftUp; i < MotoNumber; i++)
				{
					pid_ast(&pid_chassis[i],chassis.target[i],moto_chassis[i].speed_rpm);
					chassis.current [i] = (int16_t)pid_chassis[i].ctrOut;	
				}
				if(chassis.mode == CHASSIS_RELAX)
				{
						for(int i=0;i<6;i++)
						{
							memset(&moto_chassis[i],0,sizeof(moto_param));
							PID_struct_init(&pid_chassis[i],6000,15000,0,0,0);              //          
						}
				}
				send_chassis_cur(0x200,chassis.current[MotoLeftUp],chassis.current[MotoRightUp],chassis.current[MotoLeftDown],chassis.current[MotoRightDown]);
				send_chassis_cur(0x1ff,chassis.current[MotoMidUp],chassis.current[MotoMidDown],0,0);		
			taskEXIT_CRITICAL();
			osDelayUntil(&ChassisHandleLastWakeTime,CHASSIS_THREAD_PERIOD);			
  }
  /* USER CODE END pid_handle_task */
}

static void mecanum_algorithm(float vx,float vy, float vw,int16_t speed[])
{
   static float Buffer[6];
    Buffer[0] = vx + vy + vw;
    Buffer[1] = vx - vy - vw;
    Buffer[2] = vx - vy + GIMBAL_POSITION_RATIO*vw;
    Buffer[3] = vx + vy - GIMBAL_POSITION_RATIO*vw;	
    Buffer[4] = vx + vw; 
    Buffer[5] = vx - vw;
	  /* Because  the difference CC and C*/
	  speed[0] =  Buffer[0];
    speed[1] = -Buffer[1];
    speed[2] =  Buffer[2];
    speed[3] = -Buffer[3];			
	  speed[4] =  Buffer[4];
	  speed[5] = -Buffer[5];	
}
	
static void chassis_algorithm(rc_info_t *_rc , chassis_t *_chassis)
{

  /* the angle which is calculating by using the atan2 function between Vx and Vy */
	static float atan_angle = 0.0f;
	/* angle diff between gimbal and chassis */
	static float diff_angle = 0.0f;
	/* the merge spd by sqrt(Vx^2 + Vy^2) */
	static float merge_spd = 0.0f;
	
		_chassis->vx = (_rc->ch1)*NORMAL_PARAM + _chassis->UpStairVx;	
		_chassis->vy = (_rc->ch0)*NORMAL_PARAM; 
		#ifndef   PID_LINKAGE
		_chassis->vw = (_rc->ch2)*NORMAL_PARAM*1.2f;   	
		#else
		merge_spd = sqrt(_chassis->vx * _chassis->vx + _chassis->vy * _chassis->vy);
		atan_angle = atan2(_chassis->vy, _chassis->vx);
		atan_angle *= 180.0f/3.1415926f;
		diff_angle = /*yaw.BackAngle - */-(_chassis->angle - GIMBAL_MIDDLE_ANGLE); 
		diff_angle += atan_angle;	
		/* calcualate the atan_angle every time */
		diff_angle = diff_angle - (int)(diff_angle / 360.0f) * 360.0f;
		//printf("diff_angle = %f\r\n", diff_angle);
		diff_angle *= 3.1415926f/180.0f;
		_chassis->vx = merge_spd * cosf(-diff_angle);
		_chassis->vy = -merge_spd * sinf(-diff_angle);
		#endif
		mecanum_algorithm(_chassis->vx,_chassis->vy,_chassis->vw,_chassis->target);	
}

static void chassis_link_handle(void)
{
	#ifdef  PID_LINKAGE
	
		#if  1
			pid_adjust(&pid_link_out,_kp,_ki,_kd);
			pid_adjust(&pid_link_in,_kkp,_kki,_kkd);
			pid_link_out.errILim = 400;
			pid_link_out.MaxOut = maxout1;
			pid_link_in.errILim = 5000;	
			pid_link_in.MaxOut   = maxout2;  	  
		#endif	
	                     /*目标角度，云台、底盘垂直时记录的yaw电机码盘值   yaw电机实时反馈的码盘值*/
		pid_ast(&pid_out[CGLink],  GIMBAL_MIDDLE_ECD + chassis.CorgiAngle , moto_gimbal[MotoYaw].total_angle);
												 /*外环输出量  底盘陀螺仪反馈绕Z轴旋转的角速度*/	
		pid_ast(&pid_in[LinkNormal], pid_out[CGLink].ctrOut, 0);

		chassis.vw =  pid_link_in.ctrOut;
	
	#endif
	corgi_mode_ctrl(chassis.CorgiAngle,chassis.CorgiVw,chassis.CorgiMode);
	chassis_algorithm(&rc,&chassis);
}
/**
  * @brief  扭腰控制
  * @param  扭腰幅度  扭腰速度  扭腰模式 (0 固定时间 1绕云台固定角度 2退出)
  * @retval void
  * @attention  测试版    2019/12/6
  */
static void corgi_mode_ctrl(int16_t Corgi_Angle, \
	                                  int16_t sRevolMax, \
                                    uint8_t Corgi_Mode)
{
	 switch (Corgi_Mode)
	 {
		 case CORGI_BY_TIME:
		 {
		    chassis.corgi_cnt ++;
			  if((chassis.corgi_init_flag == 0)&&(chassis.corgi_cnt >= GORGI_PERIOD*0.5/CHASSIS_THREAD_PERIOD))
				{
				  sRevolMax = 8000;
					chassis.corgi_cnt = 0;	
					chassis.corgi_init_flag = 1;
				}
				else
				{
						if(chassis.corgi_cnt >= GORGI_PERIOD/CHASSIS_THREAD_PERIOD)
						{
							sRevolMax = 8000;
							chassis.corgi_cnt = 0;
						}
						else if (chassis.corgi_cnt == 0)
						{
							sRevolMax = -8000;
						}		
			  }				
		 }break;
		 /*  */
		 case CORGI_BY_GM:
		 {
		   if(chassis.corgi_flag)
			 {
			    Corgi_Angle = GORGI_GM_ANGLE;
				  if (fabs(pid_out[CGLink].errNow) <= GORGI_GM_OFFSET)
						chassis.corgi_flag	= !chassis.corgi_flag;		
			 }
			 else
			 {
			    Corgi_Angle = - GORGI_GM_ANGLE;	
				  if (fabs(pid_out[CGLink].errNow) <= GORGI_GM_OFFSET)
						chassis.corgi_flag	= !chassis.corgi_flag;					  
			 }
		 }break;	
		 		 
		 case STOP_CORGI:
		 {
				  sRevolMax = 0;
			    Corgi_Angle = 0;
					chassis.corgi_cnt = 0;				    
					chassis.corgi_init_flag = 0;		 
		 }break;			 
	 }
  
}




