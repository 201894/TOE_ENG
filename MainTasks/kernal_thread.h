
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
#define SLIP_POS_STATE   slip_accuracy(0.5f)
#define UPLIFT_POS_STATE   uplift_accuracy(2.0f)
typedef enum
{
  MANUAL_CTRL_MODE = 0,
	AUTO_FETCH_MODE1,
	AUTO_FETCH_MODE2,	
  SEMI_AUTO_MODE,
  AUTO_CTRL_MODE,
  SAFETY_MODE,
} global_mode_e;

typedef  struct
{
	uint8_t fetchMode;
  uint8_t slipPosFlag; // 滑移电机位置 标志位 u触碰开关时为1 否则为0
	uint8_t stopFlag;	
	uint8_t upLiftPosFlag;
	uint8_t upLiftSelfLockFlag;	
	float	slipPos;
	float flipAngle;	

	int16_t fricSpeed;
  global_mode_e global_mode; // 总模式控制
} kernal_ctrl_t;

uint8_t slip_accuracy(float error);
uint8_t uplift_accuracy(float error);
uint8_t flip_accuracy(float error);
static void fric_speed_ctrl(void);
static void get_main_ctrl_mode(void);
static void global_mode_init(void);
static void global_mode_handle(void);
static void chassis_mode_handle(void);
static void gimbal_mode_handle(void);
static void stir_freq_ctrl(void);

void PID_InitArgument(void);
void MODE_InitArgument(void);
extern kernal_ctrl_t kernal_ctrl;
#endif