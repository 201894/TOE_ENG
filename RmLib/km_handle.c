
/** @file km_handle.c
 *  @version 2.0
 *  @date 12.5 2019
 *
 *  @brief get km information and handle 
 *
 *
 */
 

#include "user_lib.h"
#include "km_handle.h"
#include "kernal_thread.h"

/* direction X move speed max */
#define MOVE_SPEED_X  8000   
/* direction Y move speed max */
#define MOVE_SPEED_Y  8000  
/* mouse button long press time */
#define LONG_PRESS_TIME  1000  //ms
/* key acceleration time */
#define KEY_ACC_TIME     1000  //ms
kb_ctrl_t km;

ramp_t dx_ramp = RAMP_GEN_DAFAULT;
ramp_t dy_ramp = RAMP_GEN_DAFAULT;


void km_global_hook(void)
{	
  km_chassis_hook();
	km_gimbal_hook();
	km_shoot_hook();
	mode_reverse_ctrl(rc.kb.key_code);
	lrc = rc;
}

// ·À¶¶´¦Àí
static int key_hook(uint16_t key){
	return (rc.kb.key_code  == key) && !(lrc.kb.key_code == key);
} 


static void mode_reverse_ctrl(uint16_t key)
{
   if (key_hook(key))
	 {
		switch (key)
		{
	    case KEY_R :
			{
			  
			}break;
	    case KEY_E:
			{
			  
			}break;
	    case KEY_X:
			{
			  
			}break;
	    case KEY_Z:
			{
			  
			}break;			
	 
		}
	 }
}


void key_fsm(kb_state_e *sta, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)		  
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
        if (sta == &km.lk_sta)
          km.lk_cnt = 0;
        else if(sta == &km.rk_sta)
          km.rk_cnt = 0;
        else if(sta == &km.wk_sta)
          km.wk_cnt = 0;	
        else if(sta == &km.ak_sta)
          km.ak_cnt = 0;	
        else if(sta == &km.sk_sta)
          km.sk_cnt = 0;		
        else if(sta == &km.dk_sta)
          km.dk_cnt = 0;			
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
        if (sta == &km.lk_sta)
        {
          if (km.lk_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.rk_sta)
        {
          if (km.rk_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.wk_sta)
        {
          if (km.wk_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.ak_sta)
        {
          if (km.ak_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else if(sta == &km.sk_sta)
        {
          if (km.sk_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }	
        else if(sta == &km.dk_sta)
        {
          if (km.dk_cnt++ > LONG_PRESS_TIME/KERNAL_THREAD_PERIOD)
            *sta = KEY_PRESS_LONG;
        }		
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
	
    default:
    break;      
  }
}

static void move_speed_ctrl(uint8_t fast, uint8_t slow)
{
  if (fast)
  {
    km.move = FAST_MODE;
    km.x_spd_limit = 1.3f * MOVE_SPEED_X;
    km.y_spd_limit = 1.3f * MOVE_SPEED_Y;
  }
  else if (slow)
  {
    km.move = SLOW_MODE;
    km.x_spd_limit = 0.7f * MOVE_SPEED_X;
    km.y_spd_limit = 0.7f * MOVE_SPEED_Y;
  }
  else
  {
    km.move = NORMAL_MODE;
    km.x_spd_limit = MOVE_SPEED_X;
    km.y_spd_limit = MOVE_SPEED_Y;
  }
}

static void move_direction_ctrl(uint8_t forward, uint8_t back,
	                              uint8_t left, uint8_t right)
{
  if (forward)
  {
    km.vx = km.x_spd_limit * ramp_calc(&dx_ramp);
  }
  else if (back)
  {
    km.vx = -km.x_spd_limit * ramp_calc(&dx_ramp);
  }
  else
  {
    km.vx = 0;
    ramp_init(&dx_ramp, KEY_ACC_TIME/KERNAL_THREAD_PERIOD);
  }	
	
  if (left)
  {
    km.vy = km.y_spd_limit * ramp_calc(&dy_ramp);
  }
  else if (right)
  {
    km.vy = -km.y_spd_limit * ramp_calc(&dy_ramp);
  }
  else
  {
    km.vy = 0;
    ramp_init(&dy_ramp, KEY_ACC_TIME/KERNAL_THREAD_PERIOD);
  }
}

static void km_chassis_hook(void)
{  
  move_speed_ctrl(FAST_SPD,SLOW_SPD);
	move_direction_ctrl(FORWARD,BACK,LEFT,RIGHT);  
}

static void km_gimbal_hook(void)
{
  
}

static void km_shoot_hook(void)
{
  
}

void km_global_init(void)
{
  	memset(&km,0,sizeof(kb_ctrl_t));	
}






