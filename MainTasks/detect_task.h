
/** @file detect_task.h
 *  @version 1.1
 *  @date Feb 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#ifndef __DETECT_TASK_H__
#define __DETECT_TASK_H__

#include "stm32f4xx_hal.h"
#include "RMsystem.h"
/* detect task period time (ms) */
#define DETECT_TASK_PERIOD 50

typedef enum
{
  BOTTOM_DEVICE            = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE     = 8,
  PC_SYS_OFFLINE            = 9,
  GIMBAL_YAW_OFFLINE   = 10,
  GIMBAL_BPIT_OFFLINE   = 11,
  GIMBAL_SPIT_OFFLINE   = 12,
  BASKET_MOTOR_OFFLINE  = 13,
  FRIC_MOTOR_OFFLINE    = 14,
  STIR_M1_OFFLINE           = 15,	
  STIR_M3_OFFLINE           = 16,  
  BULLET_JAM                     = 17,
  CHASSIS_CONFIG_ERR   = 18,
  GIMBAL_CONFIG_ERR    = 19,
  ERROR_LIST_LENGTH    = 20,
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

extern global_err_t g_err;
void detector_init(void);
void err_detector_hook(int err_id);

void detector_param_init(void);
static void module_offline_callback(void);
static void module_offline_detect(void);
#endif