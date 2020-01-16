
/** @file gimbal_task.h
 *  @version 1.0
 *  @date Dec 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"

#define GIMBAL_THREAD_PERIOD     5

typedef enum
{
	
	VV_SHIFT_TO_RIGHT = 0, // Î¢Î¢Æ«ÓÒ
	VV_SHIFT_TO_LEFT,  // Î¢Î¢Æ«×ó
	SHIFT_TO_RIGHT,  //  Æ«ÓÒ
	SHIFT_TO_LEFT,	  // Æ«×ó
	ERROR_STATE,
	CORRECT_STATE,
} claw_state_e;
typedef enum
{
	INIT_R_ANGLE = 0,						// ÆðÊ¼ ½Ç¶È
  MIN_R_ANGLE = -20,					// Ð¶ºÉ ½Ç¶È 
	EVALVE_SELFLOCK_ANGLE = -110,    // Æø¸××ÔËø½Ç¶È
  MID_R_ANGLE = -90,			    // É¨Ãè ½Ç¶È ´¹Ö±
  MAX_R_ANGLE = -180,				// ¼ÐÈ¡ ½Ç¶È
} flip_angle_e;

typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
} gimbal_mode_e;

typedef struct
{
  float    pitTargrtAngle;
  float    yawTargrtAngle;
  int16_t    fric_speed;	
	gimbal_mode_e   mode;
}gimbal_t;

static void gimbal_pid_handle(void);
static void gimbal_pid_adjust(void);
static void gimbal_mode_switch(void);
static void gimbal_target_constrain(void);
static void gimbal_target_handle(rc_info_t *_rc , gimbal_t *_gimbal);
void gimbal_pid_init(void);
extern gimbal_t gimbal;
#endif