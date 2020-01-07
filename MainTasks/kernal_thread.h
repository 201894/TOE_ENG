
/** @file __KERNAL_THREAD_H__
 *  @version 1.0
 *  @date Dec 2019
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */

#ifndef __KERNAL_THREAD_H__
#define __KERNAL_THREAD_H__

#include "stm32f4xx_hal.h"

#define KERNAL_THREAD_PERIOD   5
#define SPD_RATIO   0.2f
#define SLIP_POS_STATE   HAL_GPIO_ReadPin(INA_GPIO_Port,INA_Pin)&&(kernal_ctrl.slipPosFlag)
typedef enum
{
  MANUAL_CTRL_MODE = 0,
	AUTO_FETCH_MODE,
  SEMI_AUTO_MODE,
  AUTO_CTRL_MODE,
  SAFETY_MODE,
} global_mode_e;

typedef  struct
{
  uint8_t 							slipPosFlag; // ���Ƶ��λ�� ��־λ u��������ʱΪ1 ����Ϊ0
  global_mode_e  global_mode; // ��ģʽ����
} kernal_ctrl_t;

static void fric_speed_ctrl(void);
static void get_main_ctrl_mode(void);
static void get_chassis_mode(void);
static void get_gimbal_mode(void);
static void chassis_mode_handle(void);
static void gimbal_mode_handle(void);
static void stir_freq_ctrl(void);

void PID_InitArgument(void);
void MODE_InitArgument(void);
extern kernal_ctrl_t kernal_ctrl;
#endif