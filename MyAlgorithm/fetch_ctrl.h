
/** @file __FETCH_CTRL_H__
 *  @version 1.0
 *  @date  2020.1.12
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */

#ifndef __FETCH_CTRL_H__
#define __FETCH_CTRL_H__

#include "stm32f4xx_hal.h"

#define  LIFT_FETCH_ANGLE   470.0f
#define  LIFT_MAX_ANGLE       560.0f
#define  LIFT_INIT_ANGLE        0.5f
#define  SLIP_INIT_ANGLE        0.0f
#define  FLIP_INIT_ANGLE        0.0

#define SLIP_POS_STATE   slip_accuracy(0.5f)
#define UPLIFT_POS_STATE   uplift_accuracy(95.0f)
#define UPLIFT_MAX_STATE   uplift_accuracy(15.0f)
uint8_t slip_accuracy(float error);
uint8_t uplift_accuracy(float error);
uint8_t flip_to_min(float error);
uint8_t flip_to_max(float error);
void chassis_state_init(void);
void UpLiftPos_Ctrl(void);
void servo_postion_ctrl(uint8_t flag);
#endif