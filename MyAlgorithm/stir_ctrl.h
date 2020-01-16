
/** @file __STIR_CTRL_C__
 *  @version 1.0
 *  @date  2020.1.12
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */

#ifndef  __STIR_CTRL_H__
#define __STIR_CTRL_H__

#include "stm32f4xx_hal.h"
#include "stdbool.h"

typedef enum
{
  NULL_SHOT = 0,
	CLEAR_SHOT,	
	SINGLE_SHOT,
	NORMAL_SHOT,
	FAN_SHOT,
  BASE_SHOT,
} shoot_mode_e;

int shoot_once(void);
int is_continue_shoot(bool is_shoot);
int shoot_freq_ctrl(bool is_shoot,uint16_t *energy);
#endif