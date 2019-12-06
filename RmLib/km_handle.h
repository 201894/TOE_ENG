/** @file km_handle.h
 *  @version 2.0
 *  @date 12.5 2019
 *
 *  @brief get rc information and handle it
 *
 *
 */

#ifndef __KM_HANDLE_H__
#define __KM_HANDLE_H__

#include "stm32f4xx_hal.h"
#include "DR16_decode.h"

/**********************************************************************************
 * bit      :      15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
 //#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 	0x0020
//#define Q 			0x0040
//#define E			0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B			0x8000
/******************************************************/
 //      speed      key


/* control key definition */
//      direction  key
#define FORWARD    (rc.kb.bit.W)
#define BACK       (rc.kb.bit.S)
#define LEFT       (rc.kb.bit.A)
#define RIGHT      (rc.kb.bit.D)
//      speed      key
#define FAST_SPD   (rc.kb.bit.SHIFT)
#define SLOW_SPD   (rc.kb.bit.CTRL)
//      function   key or mouse operate
#define TWIST_CTRL (rc.kb.bit.E)

#define KEY_V		    0x4000
#define KEY_C		    0x2000
#define KEY_X		    0x1000
#define KEY_Z		    0x0800
#define KEY_G       0x0400
#define KEY_F		    0x0200
#define KEY_R		    0x0100
#define KEY_E		    0x0080
#define KEY_Q		    0x0040
#define KEY_CTRL	  0x0020
#define KEY_SHIFT   0x0010
#define KEY_D		    0x0008
#define KEY_A		    0x0004
#define KEY_S		    0x0002
#define KEY_W		    0x0001

typedef enum 
{
  NORMAL_MODE = 0,
  FAST_MODE,
  SLOW_MODE,
} kb_move_e;

typedef enum
{
  KEY_RELEASE = 0,
  KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG,
} kb_state_e;

typedef struct
{  
  float vx;
  float vy;
  float vw;
  
  float pit_v;
  float yaw_v;	
  float x_spd_limit;
  float y_spd_limit;
  
  uint8_t kb_enable;	
	
  uint16_t lk_cnt;
  uint16_t rk_cnt;
	
  uint16_t wk_cnt;
  uint16_t ak_cnt;
	
  uint16_t sk_cnt;
  uint16_t dk_cnt;
	
  kb_state_e lk_sta;
  kb_state_e rk_sta;
  
  kb_state_e wk_sta;
  kb_state_e sk_sta;
  
  kb_state_e ak_sta;
  kb_state_e dk_sta;	
	
  kb_move_e move;
  
} kb_ctrl_t;

void key_fsm(kb_state_e *sta, uint8_t key);
void km_global_hook(void);
static void km_chassis_hook(void);
static void km_gimbal_hook(void);
static void mode_reverse_ctrl(uint16_t key);
static void km_shoot_hook(void);
void km_global_init(void);
extern kb_ctrl_t km;

#endif 
