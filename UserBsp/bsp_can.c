
/** @file bsp_can.c
 *  @version 4.0
 *  @date  June 19
 *
 *  @brief receive external can device message
 *
 */

#include "bsp_can.h"
#include "cmsis_os.h"
#include "can.h"
#include "detect_thread.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define ENCODER_ANGLE_RATIO               (8192.0f/360.0f)
#define REDUCTION_RATIO                   (36/1)
#define pi                                 3.1415926

uint8_t TxData[8];
CAN_TxHeaderTypeDef  CAN_TxHeader;

wl2data       						data2bytes;
wl4data       						data4bytes;

/* CAN2 */
/* MOTOR REVELENT */
moto_param    	moto_chassis[8];
/* CAN1 */
/* MOTOR REVELENT */
moto_param    	moto_fric[2];
moto_param    	moto_gimbal[3];

/* JUDGEMENT SYSTEM REVELENT */
gyro_param    						gyro_yaw;
ext_game_state_t          ext_game_state;
ext_game_robot_state_t    ext_game_robot_state;
ext_power_heat_data_t     ext_power_heat_data;
ext_game_robot_pos_t      ext_game_robot_pos;
ext_buff_musk_t           ext_buff_musk;
ext_shoot_data_t          ext_shoot_data;
ext_robot_hurt_t          ext_robot_hurt;

/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */  

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{

  uint8_t RxData1[8],RxData2[8];
  CAN_RxHeaderTypeDef Can1Header,Can2Header;
  if(hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Can1Header,RxData1);
    switch (Can1Header.StdId)
		{
/* 摩擦轮数据 */	 						
	    case CAN_3508_FL_ID:  	
	    case CAN_3508_FR_ID: 				
	    {
	      static uint8_t i;
        i = Can1Header.StdId - CAN_3508_FL_ID;	
        moto_fric[i].speed_rpm = (uint16_t)(RxData1[2] << 8 | RxData1[3]);
				err_detector_hook(CAN_FRIC_M1_OFFLINE);
				g_fps[FRIC].cnt ++;  
	    }break; 
/* Stir电机数据 */	 
	    case CAN_STIR_ID:  
	    {
		 		encoder_data_handle(&moto_gimbal[MotoStir],RxData1);		
				err_detector_hook(CAN_STIR_M1_OFFLINE);
				g_fps[STIR].cnt ++;				
	    }break; 			
/* Pitch 电机数据 */	 			
	    case CAN_PIT_ID:  	
	    {
		 		encoder_data_handle(&moto_gimbal[MotoPit],RxData1);	
				err_detector_hook(CAN_PIT_M1_OFFLINE);
				g_fps[PIT].cnt ++;				
	    }break; 	
/* Yaw 电机数据 */	 			
	    case CAN_YAW_ID: 				
	    {
		 		encoder_data_handle(&moto_gimbal[MotoYaw],RxData1);
				err_detector_hook(CAN_YAW_M1_OFFLINE);
				g_fps[YAW].cnt ++;				
				
	    }break; 		
/* 比赛状态数据 */	  
	    case EXT_GAME_STATE:
	    {
				ext_game_state.game_progress = RxData1[0];		  
	    }	break;
/* 比赛机器人状态 */	  
	    case EXT_ROBOT_STATE:
	    {
				ext_game_robot_state.robot_level = RxData1[0];			  		
				data2bytes.c [0] = RxData1[1];
				data2bytes.c [1] = RxData1[2];
				ext_game_robot_state.robot_id = RxData1[3];		  
				ext_game_robot_state.remain_HP = data2bytes.ud;
				
	     }	break;
/* 实时功率数据 */
	    case EXT_POWER_DATA:
	    {
				data2bytes.c[0] = RxData1[0];
				data2bytes.c[1] = RxData1[1];		  
				ext_power_heat_data.chassis_volt = data2bytes.ud;
				data2bytes.c[0] = RxData1[2];
				data2bytes.c[1] = RxData1[3];	
				ext_power_heat_data.chassis_current = data2bytes.ud;
				data4bytes.c[0] = RxData1[4];
				data4bytes.c[1] = RxData1[5];
				data4bytes.c[2] = RxData1[6];
				data4bytes.c[3] = RxData1[7];	
				ext_power_heat_data.chassis_power = data4bytes.f ;	       	
	    }	break;
/* 实时热量数据 */
	    case EXT_HEAT_DATA:
	    {
				data2bytes.c[0] = RxData1[0];
				data2bytes.c[1] = RxData1[1];	
				ext_power_heat_data.chassis_power_buffer  = data2bytes.ud;	
				data2bytes.c[0] = RxData1[2];
				data2bytes.c[1] = RxData1[3];	
				ext_power_heat_data.shooter_heat0 = data2bytes.ud;  
				data2bytes.c[0] = RxData1[4];
				data2bytes.c[1] = RxData1[5];	
				ext_power_heat_data.shooter_heat1 = data2bytes.ud; 
				err_detector_hook(CAN_JUDGE_OFFLINE);
				g_fps[JUDGE].cnt ++;							
	    } break;
/* 机器人位置信息与枪口朝向 */
	    case EXT_XY_POSITION:
	    {		  
				data4bytes.c[0] = RxData1[0];
				data4bytes.c[1] = RxData1[1];
				data4bytes.c[2] = RxData1[2];
				data4bytes.c[3] = RxData1[3];	
				ext_game_robot_pos.x = data4bytes.f ;
				data4bytes.c[0] = RxData1[4];
				data4bytes.c[1] = RxData1[5];
				data4bytes.c[2] = RxData1[6];
				data4bytes.c[3] = RxData1[7];	
				ext_game_robot_pos.y = data4bytes.f ;	  
	    }	break;
/* 机器人位置信息与枪口朝向 */
	    case EXT_ZY_POSITION:
	    {		  
				data4bytes.c[0] = RxData1[0];
				data4bytes.c[1] = RxData1[1];
				data4bytes.c[2] = RxData1[2];
				data4bytes.c[3] = RxData1[3];	
				ext_game_robot_pos.z = data4bytes.f ;
				data4bytes.c[0] = RxData1[4];
				data4bytes.c[1] = RxData1[5];
				data4bytes.c[2] = RxData1[6];
				data4bytes.c[3] = RxData1[7];	
				ext_game_robot_pos.yaw = data4bytes.f ;	  
	    } break;
/* 增益信息 */			
	    case EXT_BUFF_MUSK:
	    {
				ext_buff_musk.power_rune_buff = RxData1[0];
	    } break;
/* 伤害信息 */
	    case EXT_ROBOT_HURT:
	    {
				ext_robot_hurt.armor_id = RxData1[0];
				ext_robot_hurt.hurt_type = RxData1[1];
	    }	break;
/* 实时射击信息 */	  
	    case EXT_SHOOT_DATA:
	    {	
				ext_shoot_data.bullet_type = RxData1[0];
				ext_shoot_data.bullet_freq  = RxData1[1];
				data4bytes.c[0] = RxData1[2];
				data4bytes.c[1] = RxData1[3];
				data4bytes.c[2] = RxData1[4];
				data4bytes.c[3] = RxData1[5];
				ext_shoot_data.bullet_speed = data4bytes.f;	
	    }	break;  	
			
	    default:
      {		 
      }break;
		}
  }
  if(hcan->Instance == CAN2)
  {
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&Can2Header,RxData2);
    switch (Can2Header.StdId)
		{
			
/* 抬升电机相关*/			
		  case CAN_UPLIFT_M1_ID:
			{
		 		encoder_data_handle(&moto_chassis[MotoLeftUpLift],RxData2);		
				err_detector_hook(CAN_UPLIFT_LEFT_OFFLINE);
				g_fps[LUP].cnt ++;						
			}break;								
			
	    case CAN_UPLIFT_M2_ID:
			{
		 		encoder_data_handle(&moto_chassis[MotoRightUpLift],RxData2);		
				err_detector_hook(CAN_UPLIFT_RIGHT_OFFLINE);
				g_fps[RUP].cnt ++;					
			}break;					
/*底盘电机相关id*/			
	    case CAN_3508_M1_ID:  
	    case CAN_3508_M2_ID:  
	    case CAN_3508_M3_ID:  
	    case CAN_3508_M4_ID:
	    case CAN_3508_M5_ID:  			
	    case CAN_3508_M6_ID:  				
	    {
	       static uint8_t i;
         i = Can2Header.StdId - CAN_3508_M1_ID;			
         moto_chassis[i].speed_rpm = (uint16_t)(RxData2[2] << 8 | RxData2[3]);			 	
				err_detector_hook(CAN_CHASSIS_OFFLINE);
				g_fps[CHASSIS].cnt ++;							
	    }break;  	
/* 从机信息相关*/		
	    case CAN_SLAVE_M1_ID:  
			{
				
				err_detector_hook(CAN_COMMU_OFFLINE);
				g_fps[COMMU].cnt ++;			
				
			}break;		
	    case CAN_SLAVE_M2_ID:  
			{
				
			}break;					
			default:
      { 
      }break;
    }
	}	   
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */

void encoder_data_handle(moto_param* ptr,uint8_t RxData[8])
{
   ptr->last_ecd = ptr->ecd;
	 if(ptr->init_flag == 0)
	 {
		 ptr->init_flag = 1;
		 ptr->round_cnt = 0;		 
	   ptr->offset_ecd = (uint16_t)(RxData[0] << 8 | RxData[1]);  
	   ptr->offset_angle = ptr->offset_ecd/ENCODER_ANGLE_RATIO;  		 
	 }
	 else
	 {
		 ptr->ecd      = (uint16_t)(RxData[0] << 8 | RxData[1]);  
		 if (ptr->ecd - ptr->last_ecd > 4096)
		 {
			 ptr->round_cnt--;
		 }
		 else if (ptr->ecd - ptr->last_ecd < -4096)
		 {
			 ptr->round_cnt++;
		 }
		 ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
		 /* total angle, unit is degree */
		 ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO; 
		 ptr->speed_rpm     = (int16_t)(RxData[2] << 8 | RxData[3]);
		 ptr->current = (int16_t)(RxData[2] << 8 | RxData[3]);
	 }
}

/**
  * @brief     get gyro data and unpack the data 
* @param     ptr: Pointer to a wl2data structure  ptrr: Pointer to a wl4data structure
  * @attention this function should be called after gyro is read
  */
void gyro_data_handle(wl2data* ptr,wl4data* ptrr,gyro_param* gyro,uint8_t RxData[8])
{
	  ptr->c[0] = RxData[0];
		ptr->c[1] = RxData[1];
	  gyro->pit_speed = -(float)ptr->d / 16.384f;
	   
		ptr->c[0] = RxData[2];
		ptr->c[1] = RxData[3];    
		gyro->yaw_speed  = -(float)ptr->d / 16.384f;

		ptrr->c[0] = RxData[4];
		ptrr->c[1] = RxData[5];
		ptrr->c[2] = RxData[6];
		ptrr->c[3] = RxData[7];
		gyro->angle_z = ptrr->f ;	
}

void send_gimbal_cur(uint32_t id,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	  CAN_TxHeader.StdId    = id;
	  CAN_TxHeader.IDE      = CAN_ID_STD;
	  CAN_TxHeader.RTR     = CAN_RTR_DATA;
	  CAN_TxHeader.DLC     = 0x08;
		if (id == 0x1FF)
		{
			TxData[0] = iq1 >> 8;
			TxData[1] = iq1;
			TxData[2] = iq2 >> 8;
			TxData[3] = iq2;
			TxData[4] = iq3 >> 8;
			TxData[5] = iq3;
			TxData[6] = iq4 >> 8;
			TxData[7] = iq4;
		}
		else
		{
			TxData[6] = iq4 >> 8;
			TxData[7] = iq4;			
		}
    HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeader,TxData,(uint32_t *)CAN_TX_MAILBOX0);
}
/**
  * @brief  send calculated current to motor
  * @param  3508 motor ESC id
  */

void send_chassis_cur(uint32_t id,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	  CAN_TxHeader.StdId    = id;
	  CAN_TxHeader.IDE      = CAN_ID_STD;
	  CAN_TxHeader.RTR      = CAN_RTR_DATA;
	  CAN_TxHeader.DLC      = 0x08;
	  TxData[0] = iq1 >> 8;
	  TxData[1] = iq1;
	  TxData[2] = iq2 >> 8;
	  TxData[3] = iq2;
	  TxData[4] = iq3 >> 8;
	  TxData[5] = iq3;
	  TxData[6] = iq4 >> 8;
	  TxData[7] = iq4;
    HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeader,TxData,(uint32_t *)CAN_TX_MAILBOX0);
}

void send_can2_ms(uint32_t id,uint8_t data[8])
{
	  CAN_TxHeader.StdId    = id;
	  CAN_TxHeader.IDE      = CAN_ID_STD;
	  CAN_TxHeader.RTR     = CAN_RTR_DATA;
	  CAN_TxHeader.DLC     = 0x08;
	  TxData[0] = data[0];
	  TxData[1] = data[1];
	  TxData[2] = data[2];
	  TxData[3] = data[3];
	  TxData[4] = data[4];
	  TxData[5] = data[5];
	  TxData[6] = data[6];
	  TxData[7] = data[7];
    HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeader,TxData,(uint32_t *)CAN_TX_MAILBOX0);
}

void can_device_init(void)
{
  //can1 &can2 use same filter config
  CAN_FilterTypeDef  can_filter;
	
  can_filter.FilterActivation     = ENABLE;
  can_filter.FilterBank           = 14U;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);
  //while (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK);
  /* Filter 1 : Four ID */
  can_filter.FilterActivation     = ENABLE;	
  can_filter.FilterBank         	= 0U;
	can_filter.FilterMode 					= CAN_FILTERMODE_IDLIST;//列表模式
	can_filter.FilterScale 					= CAN_FILTERSCALE_16BIT;//16位宽
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterIdHigh 				= ((uint16_t)CAN_STIR_ID)<<5 ;
	can_filter.FilterIdLow 					= ((uint16_t)CAN_PIT_ID)<<5;
	can_filter.FilterMaskIdHigh 		= ((uint16_t)CAN_YAW_ID)<<5;
	can_filter.FilterMaskIdLow 			= ((uint16_t)CAN_3508_FL_ID)<<5;	
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);	
  /* Filter 2 : Four ID */
  can_filter.FilterActivation 		= ENABLE;	
  can_filter.FilterBank 					= 1U;
	can_filter.FilterMode 					= CAN_FILTERMODE_IDLIST;//列表模式
	can_filter.FilterScale 					= CAN_FILTERSCALE_16BIT;//16位宽
	can_filter.FilterFIFOAssignment 			= CAN_FILTER_FIFO0;
	can_filter.FilterIdHigh 				= ((uint16_t)CAN_3508_FR_ID)<<5 ;
	can_filter.FilterIdLow 					= ((uint16_t)CAN_SLAVE_M1_ID)<<5;
	can_filter.FilterMaskIdHigh 		= ((uint16_t)CAN_SLAVE_M2_ID)<<5;
	can_filter.FilterMaskIdLow 			= ((uint16_t)0)<<5;		
  HAL_CAN_ConfigFilter(&hcan1, &can_filter);		
}

void can_receive_start(void)
{	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_InitArgument(void)
{
		can_device_init();
		can_receive_start();
}

