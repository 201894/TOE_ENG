
/** @file __CHASSIS_THREAD_H__
 *  @version 1.1
 *  @date Dec 2019
 *
 *  @brief handle the chassis move and pid handle
 *
 */
 

#include "chassis_thread.h"
#include "detect_thread.h"
#include "kernal_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "STMGood.h"
#include "pid.h"
#include "string.h"
#include "math.h"
#include "user_lib.h"
#include "bsp_io.h"

#define CHASSIS_THREAD_PERIOD 	  	2							// ��������
#define GIMBAL_POSITION_RATIO		 		2.0f					// ��̨ ���� λ�ñ���
#define GIMBAL_MIDDLE_ANGLE   			100.0f      // ��̨��ֵ�Ƕ�
#define GIMBAL_MIDDLE_ECD     			 		4000				 // ��̨��ֵ����ֵ
#define NORMAL_PARAM						      	 	12.12f			 // ң���� - ���� ��Ӧ���� 
#define CHASSIS_VX_STOP                	600.0f				 //ȡ��ģʽ ����VX����
/*Ť��ģʽ��ز���*/
#define GORGI_PERIOD                       1600      /* ��λʱ��ģʽ    Ť������ */
#define GORGI_GM_ANGLE                 40.0f			/* ��̨���ģʽ    Ť������ */
#define GORGI_GM_OFFSET               8.0f
/*���̵�� PID ����*/
#define  CHASSIS_MAX_CURRENT        		15000 // 
#define  CHASSIS_SPD_ERRORLLIM       	6000 // 
#define  CHASSIS_SPD_KP                    		7.0f // 
#define  CHASSIS_SPD_KI                      	  0.1f // 
#define  CHASSIS_SPD_KD                    		0.0f // 
/*̧����� PID ����*/
#define  LIFT_MAX_SPD                  	800 // 
#define  LIFT_ANG_ERRORLLIM     	3000 // 
#define  LIFT_MAX_CURRENT        		15000 // 
#define  LIFT_SPD_ERRORLLIM       	6000 // 
#define  LIFT_ANG_KP                    	32.0f // 
#define  LIFT_SPD_KP                    		40.0f // 
#define  LIFT_SPD_KI                      	0.25f // 
#define  LIFT_SPD_KD                    		0.0f // 
/*���̸��� PID ����*/
#define  LINK_MAX_SPD                  	1200 // 
#define  LINK_ANG_ERRORLLIM     	3000 // 
#define  LINK_MAX_CURRENT        	15000 // 
#define  LINK_SPD_ERRORLLIM       6000 // 
#define  LINK_ANG_KP                    	32.0f // 
#define  LINK_SPD_KP                    	40.0f // 
#define  LINK_SPD_KI                      	0.25f // 
#define  LINK_SPD_KD                    	0.0f // 


chassis_t chassis;

void chassis_pid_init(void)
{
	for(int i=0;i<6;i++)
	{
	  memset(&moto_chassis[i],0,sizeof(moto_param));
	  PID_struct_init(&pid_chassis[i],CHASSIS_SPD_ERRORLLIM,CHASSIS_MAX_CURRENT,CHASSIS_SPD_KP,CHASSIS_SPD_KI,CHASSIS_SPD_KD);              //          
	} 
  PID_struct_init(&pid_out[LinkECD],0,0,0,0,0);  	
  PID_struct_init(&pid_in[CGLink],0,0,0,0,0);  
	PID_struct_init(&pid_out[LiftECD],LIFT_ANG_ERRORLLIM,LIFT_MAX_SPD,LIFT_ANG_KP,0,0); 	
  PID_struct_init(&pid_in[MotoLUpLft],LIFT_SPD_ERRORLLIM,LIFT_MAX_CURRENT,LIFT_SPD_KP,LIFT_SPD_KI,LIFT_SPD_KD);
  PID_struct_init(&pid_in[MotoRUpLft],LIFT_SPD_ERRORLLIM,LIFT_MAX_CURRENT,LIFT_SPD_KP,LIFT_SPD_KI,LIFT_SPD_KD);	
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
#if 0		
				for(int i=0;i<6;i++)
				{
					PID_struct_init(&pid_chassis[i],6000,maxout1,_kp,_ki,_kd);              //          
				}		
#endif				
				chassis_link_handle();
				for(int i = MotoLeftUp; i < MotoNumber; i++)
				{
					pid_ast(&pid_chassis[i],chassis.target[i],moto_chassis[i].speed_rpm);
					chassis.current[i] = (int16_t)pid_chassis[i].ctrOut;	
				}
#if 0
			PID_struct_init(&pid_out[LiftECD],3000,maxout1,_kp,_ki,_kd);  
			PID_struct_init(&pid_in[MotoLUpLft],6000,15000,_kkp,_kki,_kkd);
			PID_struct_init(&pid_in[MotoRUpLft],6000,15000,_kkp,_kki,_kkd);			
#endif
		/* ̧����� �� ��ת��� ���Ʋ��� ��		
		    ʹ��һ��������⻷���ֵ����Ϊ�ڻ����Ƶ�Ŀ��ֵ
		*/
				chassis_target_constrain();
		/* ̧�����PID����    �⻷   ����Ŀ��λ��           --        �������λ��  */				
				pid_ast(&pid_out[LiftECD], chassis.targetPosition, moto_chassis[MotoLeftUpLift].total_angle);
		/* ̧�����PID����    �ڻ�   �⻷�����             --        ��������ٶ�  */		
				pid_ast(&pid_in[MotoLUpLft],pid_out[LiftECD].ctrOut,moto_chassis[MotoLeftUpLift].speed_rpm * SPD_RATIO); 
				pid_ast(&pid_in[MotoRUpLft],-pid_out[LiftECD].ctrOut,moto_chassis[MotoRightUpLift].speed_rpm * SPD_RATIO);
		/* ����� ��� �ջ� ����ֵ ����    ̧����� 201 202   ���̵�� 203 ~ 208 */						
				if(chassis.canSendFlag == SET)
				{
					send_chassis_cur(0x200,(int16_t)pid_in[MotoLUpLft].ctrOut,(int16_t)pid_in[MotoRUpLft].ctrOut,chassis.current[MotoLeftUp],chassis.current[MotoRightUp]);
						send_chassis_cur(0x1ff,chassis.current[MotoLeftDown],chassis.current[MotoRightDown],chassis.current[MotoMidUp],chassis.current[MotoMidDown]);						
				}
				else
				{
					send_chassis_cur(0x200,0,0,0,0);
					send_chassis_cur(0x1ff,0,0,0,0);				
				}
			taskEXIT_CRITICAL();
			osDelayUntil(&ChassisHandleLastWakeTime,CHASSIS_THREAD_PERIOD);			
  }
  /* USER CODE END pid_handle_task */
}
static void chassis_target_constrain(void)
{
	chassis.targetPosition = float_constrain(chassis.targetPosition,0,564);
}
static void mecanum_algorithm(float vx,float vy, float vw,int16_t speed[])
{
   static float Buffer[6];
    Buffer[0] = vx + vy + vw;
    Buffer[1] = vx - vy - vw;
    Buffer[2] = vx - vy + vw;	
    Buffer[3] = vx + vy - vw;	
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
	
		_chassis->vx = (_rc->ch1)*NORMAL_PARAM + _chassis->upStairVx;	
		_chassis->vy =-(_rc->ch0)*NORMAL_PARAM; 
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
		if (_chassis->stopFlag == 0)
			mecanum_algorithm( _chassis->vx, _chassis->vy, _chassis->vw, _chassis->target);	
		else{
//			kernal_ctrl.flipAngle -= (float)(_rc->ch3)*0.002;
//			_chassis->targetPosition  += (float)(_rc->ch1)*0.002;			
			mecanum_algorithm(CHASSIS_VX_STOP, 0, 0, _chassis->target);	
		}			
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
	                     /*Ŀ��Ƕȣ���̨�����̴�ֱʱ��¼��yaw�������ֵ   yaw���ʵʱ����������ֵ*/
		pid_ast(&pid_out[CGLink],  GIMBAL_MIDDLE_ECD + chassis.CorgiAngle , moto_gimbal[MotoYaw].total_angle);
												 /*�⻷�����  ���������Ƿ�����Z����ת�Ľ��ٶ�*/	
		pid_ast(&pid_in[LinkECD], pid_out[CGLink].ctrOut, 0);

		chassis.vw =  pid_link_in.ctrOut;
	
	#endif
	corgi_mode_ctrl(chassis.corgiAngle,chassis.corgiVw,chassis.corgiMode);
	chassis_algorithm(&rc,&chassis);
}
/**
  * @brief  Ť������
  * @param  Ť������  Ť���ٶ�  Ť��ģʽ (0 �̶�ʱ�� 1����̨�̶��Ƕ� 2�˳�)
  * @retval void
  * @attention  ���԰�    2019/12/6
  */
static void corgi_mode_ctrl(int16_t Corgi_Angle, \
	                                  int16_t sRevolMax, \
                                    uint8_t Corgi_Mode)
{
	 switch (Corgi_Mode)
	 {
		 case CORGI_BY_TIME:
		 {
		    chassis.corgiCnt ++;
			  if((chassis.corgiInitFlag == 0)&&(chassis.corgiCnt >= GORGI_PERIOD*0.5/CHASSIS_THREAD_PERIOD))
				{
				  sRevolMax = 8000;
					chassis.corgiCnt = 0;	
					chassis.corgiInitFlag = 1;
				}
				else
				{
						if(chassis.corgiCnt >= GORGI_PERIOD/CHASSIS_THREAD_PERIOD)
						{
							sRevolMax = 8000;
							chassis.corgiCnt = 0;
						}
						else if (chassis.corgiCnt == 0)
						{
							sRevolMax = -8000;
						}		
			  }				
		 }break;
		 /*  */
		 case CORGI_BY_GM:
		 {
		   if(chassis.corgiFlag)
			 {
			    Corgi_Angle = GORGI_GM_ANGLE;
				  if (fabs(pid_out[CGLink].errNow) <= GORGI_GM_OFFSET)
						chassis.corgiFlag	= !chassis.corgiFlag;		
			 }
			 else
			 {
			    Corgi_Angle = - GORGI_GM_ANGLE;	
				  if (fabs(pid_out[CGLink].errNow) <= GORGI_GM_OFFSET)
						chassis.corgiFlag	= !chassis.corgiFlag;					  
			 }
		 }break;	
		 		 
		 case STOP_CORGI:
		 {
				  sRevolMax = 0;
			    Corgi_Angle = 0;
					chassis.corgiCnt = 0;				    
					chassis.corgiInitFlag = 0;		 
		 }break;			 
	 }
  
}




