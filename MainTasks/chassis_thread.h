
/** @file __CHASSIS_THREAD_H__
 *  @version 1.1
 *  @date Dec 2019
 *
 *  @brief handle the chassis move and pid handle
 *
 */

#ifndef __CHASSIS_THREAD_H__
#define __CHASSIS_THREAD_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"

typedef enum
{	
  CORGI_BY_TIME ,
  CORGI_BY_GM,
  STOP_CORGI,	
}corgi_mode_e;

typedef enum
{	
  CHASSIS_RELAX = 0,
  CHASSIS_STOP = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL = 3,
  DODGE_MODE = 4,
  AUTO_SEPARATE_GIMBAL = 5,
  AUTO_FOLLOW_GIMBAL = 6,
	UPSTAIR_MODE = 7,
} chassis_mode_e;

typedef  struct
{
  float         							vx; // forward/back
  float         							vy; // left/right
  float         							vw; // 
	float         							angle;
  float         							upStairVx;	
  float         							corgiAngle;	
	float 											targetPosition;   // 抬升位置 (电机角度环目标)	
	uint8_t               		stopFlag;	
	uint8_t               		canSendFlag;
	uint8_t                   corgiInitFlag;		

	uint8_t                   corgiFlag;	
	uint16_t                 corgiCnt;
  int16_t        						corgiVw;  	
  int16_t       						current[6];
  int16_t       						target[6];	
  chassis_mode_e		mode;
	corgi_mode_e       corgiMode;
}chassis_t;

static void chassis_link_handle(void);
static void chassis_pid_handle(void);
static void mecanum_algorithm(float vx,float vy, float vw,int16_t speed[]);
static void chassis_algorithm(rc_info_t *_rc , chassis_t *_chassis);
static void corgi_mode_ctrl(int16_t Corgi_Angle, \
	                                  int16_t sRevolMax, \
                                    uint8_t Corgi_Mode); //扭腰模式
void chassis_pid_init(void);
extern chassis_t chassis;
#endif