
/** @file detect_thread.h
 *  @version 1.1
 *  @date Feb 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#ifndef __DETECT_THREAD_H__
#define __DETECT_THREAD_H__

#include "stm32f4xx_hal.h"
#include "bsp_can.h"
/* detect task period time (ms) */

typedef enum
{	
   STIR,
	 YAW,
	 PIT,
	 FRIC,
	 CHASSIS,
	 JUDGE,
	 NUC,
	 DEBUS,
	 IMU,
	 COMMU,
	 GYROY,
	 GYROP, 
	 LUP,
	 RUP,
	SLIP,
	LFLIP,
	RFLIP,
}module_fps_e;

typedef enum
{
  BOTTOM_DEVICE            					= 0,
  UART_DEBUS_OFFLINE 	  			  = 1,	  
  UART_NUC_OFFLINE 	  					  = 2,	
  CAN_JUDGE_OFFLINE  					  = 3,	
  CAN_COMMU_OFFLINE 	  	      = 4,		
  CAN_YAW_M1_OFFLINE  					= 5,
  CAN_PIT_M1_OFFLINE 	  					= 6,
  CAN_YAW_IMU_OFFLINE  				= 7,
  CAN_PIT_IMU_OFFLINE 	  			  = 8,	
  CAN_FRIC_M1_OFFLINE    				= 9,
  CAN_CHASSIS_OFFLINE    	      = 10,	
  CAN_STIR_M1_OFFLINE          	= 11,  
	CAN_UPLIFT_LEFT_OFFLINE   = 12,
	CAN_UPLIFT_RIGHT_OFFLINE = 13,
  CAN_SLIP_M1_OFFLINE          	= 14,  
	CAN_FLIP_LEFT_OFFLINE   			= 15,
	CAN_FLIP_RIGHT_OFFLINE 			= 16,	
  BULLET_JAM               							= 17,
  CHASSIS_CONFIG_ERR   		 			= 18,
  GIMBAL_CONFIG_ERR   	 					= 19,
  ERROR_LIST_LENGTH    					= 20,
} err_id_e;

typedef enum
{
  DEV_OFFLINE     = 0,
  DEV_RUNNING_ERR = 1,
  SYS_CONFIG_ERR  = 2,
} err_type_e;

typedef struct
{
  uint16_t set_timeout;
  uint16_t delta_time;
  uint32_t last_time;
} offline_dev_t;

typedef struct
{
  /* enable the device error detect */
  uint8_t  enable;
  /* device error exist flag */
  uint8_t  err_exist;
  /* device error priority */
  uint8_t  pri;
  /* device error type */
  uint8_t  type;
  /* the pointer of device offline param */
  offline_dev_t *dev;
} err_dev_t;

typedef struct
{
  /* the pointer of the highest priority error device */
  err_dev_t *err_now;
  err_id_e  err_now_id;
  /* all device be detected list */
  err_dev_t list[ERROR_LIST_LENGTH];
  /* error alarm relevant */
  uint16_t err_count;
  uint16_t beep_tune;
  uint16_t beep_ctrl;
} global_err_t;

typedef struct
{
  /* error alarm relevant */
  uint16_t fps;
  uint16_t cnt;
  uint16_t beep_ctrl;
	
} global_fps_t;

extern global_err_t g_err;
extern global_fps_t g_fps[MaxId];
extern global_fps_t r_fps[MaxId];
void detector_init(void);
void OLED_CTRL(void);
void err_detector_hook(int err_id);
void slove_ms_send(uint8_t mode, \
																	uint8_t flag1, \
																		uint8_t flag2, \
																			uint8_t flag3, \
																				float targrt_angle);
void DETECT_InitArgument(void);
static void module_offline_callback(void);
static void module_offline_detect(void);
static void module_fps_detect(void);
static void module_fps_clear(void);
#endif