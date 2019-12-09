
/** @file detect_task.c
 *  @version 1.0
 *  @date Feb 2019
 *
 *  @brief detect module offline or online task
 *
 *
 */

#include "detect_thread.h"
#include "cmsis_os.h"
#include "bsp_io.h"
#include "freertos.h"
#include "tim.h"
#include "string.h"

#define DETECT_THREAD_PERIOD 100
UBaseType_t detect_stack_surplus;

/* detect task global parameter */
global_err_t g_err;
global_fps_t g_fps[MaxId];
/* detect task static parameter */
static offline_dev_t offline_dev[CAN_STIR_M1_OFFLINE + 1];
/**
  * @brief     initialize detector error_list
  * @usage     used before detect loop in detect_task() function
  */
void detector_init(void)
{
	 g_err.err_now = NULL;
	 g_err.list[BOTTOM_DEVICE].dev    = NULL;
	 g_err.list[BOTTOM_DEVICE].enable = 0;

	 /* initialize device error type and offline timeout value */
	 for (uint8_t i = CAN_JUDGE_OFFLINE; i < ERROR_LIST_LENGTH; i++)
	 {		  
	  if (i <= CAN_STIR_M1_OFFLINE)
		{
		  offline_dev[i].set_timeout = 200; //ms
		  offline_dev[i].last_time   = 0;
		  offline_dev[i].delta_time  = 0;		  
		  g_err.list[i].dev  = &offline_dev[i];
		  g_err.list[i].type = DEV_OFFLINE;
		}
		else if (i == BULLET_JAM)
		{
		  g_err.list[i].dev  = NULL;
		  g_err.list[i].type = DEV_RUNNING_ERR;
		}
		else if (i <= GIMBAL_CONFIG_ERR)
		{
		  g_err.list[i].dev  = NULL;
		  g_err.list[i].type = SYS_CONFIG_ERR;
		}
	 }
		
	  /* initialize device error detect priority and enable byte */
	  g_err.list[UART_DEBUS_OFFLINE].err_exist  = 0;
	  g_err.list[UART_DEBUS_OFFLINE].pri        = 11;
	  g_err.list[UART_DEBUS_OFFLINE].enable     = 1;
	 
	  g_err.list[CAN_STIR_M1_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_STIR_M1_OFFLINE].pri        = 1;
	  g_err.list[CAN_STIR_M1_OFFLINE].enable     = 1;
	  
	  g_err.list[CAN_CHASSIS_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_CHASSIS_OFFLINE].pri        = 2;
	  g_err.list[CAN_CHASSIS_OFFLINE].enable     = 1;
	  
	  g_err.list[CAN_FRIC_M1_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_FRIC_M1_OFFLINE].pri        = 3;
	  g_err.list[CAN_FRIC_M1_OFFLINE].enable     = 1;

	  g_err.list[CAN_PIT_IMU_OFFLINE].err_exist = 0;
	  g_err.list[CAN_PIT_IMU_OFFLINE].pri       = 4;
	  g_err.list[CAN_PIT_IMU_OFFLINE].enable    = 1;
	  
	  g_err.list[CAN_YAW_IMU_OFFLINE].err_exist = 0;
	  g_err.list[CAN_YAW_IMU_OFFLINE].pri          = 5;  //max priority
	  g_err.list[CAN_YAW_IMU_OFFLINE].enable    = 1;

	  g_err.list[CAN_PIT_M1_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_PIT_M1_OFFLINE].pri        = 6;
	  g_err.list[CAN_PIT_M1_OFFLINE].enable     = 1;
	  
	  g_err.list[CAN_YAW_M1_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_YAW_M1_OFFLINE].pri        = 7;
	  g_err.list[CAN_YAW_M1_OFFLINE].enable     = 1;
	  
	  g_err.list[CAN_COMMU_OFFLINE].err_exist  = 0;
	  g_err.list[CAN_COMMU_OFFLINE].pri        = 8;
	  g_err.list[CAN_COMMU_OFFLINE].enable     = 1;

	  g_err.list[CAN_JUDGE_OFFLINE].err_exist = 0;
	  g_err.list[CAN_JUDGE_OFFLINE].pri       = 9;
	  g_err.list[CAN_JUDGE_OFFLINE].enable    = 1;
	  
	  g_err.list[UART_NUC_OFFLINE].err_exist    = 0;
	  g_err.list[UART_NUC_OFFLINE].pri          = 10;
	  g_err.list[UART_NUC_OFFLINE].enable       = 1;
}  
/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_detector_hook(int err_id)
{
  if (g_err.list[err_id].enable)
      g_err.list[err_id].dev->last_time = HAL_GetTick();
}

void DETECT_InitArgument(void)
{
  detector_init();
  g_err.beep_ctrl = 0;  
  led_init;
}

/**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
uint32_t detect_time_last;
int detect_time_ms;
void detect_thread(void const *argu)
{
  LED_R_OFF;
  LED_G_OFF;
  uint32_t detect_wake_time = osKernelSysTick();
		
  while(1)
  {
    detect_time_ms = HAL_GetTick() - detect_time_last;
    detect_time_last = HAL_GetTick();
    
    /* module offline detect */
    module_offline_detect();
    if (g_err.err_now != NULL)
    {
      LED_R_OFF;
      module_offline_callback();
    }
    else
    {
      g_err.beep_ctrl = 0;
      LED_G_ON;
    }

    module_fps_detect();
    module_fps_clear();
   // detect_stack_surplus = uxTaskGetStackHighWaterMark(NULL);    
    osDelayUntil(&detect_wake_time, DETECT_THREAD_PERIOD);
  }
}
  
static void module_offline_detect(void)
{
  int max_priority = 0;
  int err_cnt      = 0;
  for (uint8_t id = CAN_JUDGE_OFFLINE; id <= CAN_STIR_M1_OFFLINE; id++)
  {
    g_err.list[id].dev->delta_time = HAL_GetTick() - g_err.list[id].dev->last_time;
    if (g_err.list[id].enable 
        && (g_err.list[id].dev->delta_time > g_err.list[id].dev->set_timeout))
    {
      g_err.list[id].err_exist = 1; //this module is offline
      err_cnt++;
      if (g_err.list[id].pri > max_priority)
      {
        max_priority     = g_err.list[id].pri;
        g_err.err_now    = &(g_err.list[id]);
        g_err.err_now_id = (err_id_e)id;
		  
      }
    }
    else
    {
      g_err.list[id].err_exist = 0;
    }
  }

  if (!err_cnt)
  {
    g_err.err_now    = NULL;
    g_err.err_now_id = BOTTOM_DEVICE;
  }
}

static void module_offline_callback(void)
{
    g_err.err_count++;
    if (g_err.err_count > 60)
    g_err.err_count = 0;

  switch (g_err.err_now_id)
  {
    case UART_DEBUS_OFFLINE:
//    case UART_NUC_OFFLINE:			
    {
      if (g_err.err_count == 1)
      {
        LED_R_ON;        
				__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);	
      }
      else
      {
				__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,0);	
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
    case CAN_CHASSIS_OFFLINE:
    {
      if (g_err.err_count == 1
          || g_err.err_count == 7)
      {
        LED_R_ON;
        g_err.beep_ctrl = g_err.beep_tune/2;
				__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
      }
      else
      {
        LED_R_OFF;
        g_err.beep_ctrl = 0;
      }
    }break;
    
//    case CAN_SLIP_OFFLINE:
//    {
//      if (g_err.err_count == 1
//          || g_err.err_count == 7
//          || g_err.err_count == 13)
//      {
//        LED_R_ON;
//        g_err.beep_ctrl = g_err.beep_tune/2;
//				__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
//      }
//      else
//      {
//        LED_R_OFF;
//        g_err.beep_ctrl = 0;
//      }
//    }break;
//    
//    case MODE_CTRL_OFFLINE:
//    {
//      if (g_err.err_count == 1
//          || g_err.err_count == 7
//          || g_err.err_count == 13
//          || g_err.err_count == 19)
//      {
//        LED_R_ON;
//        g_err.beep_ctrl = g_err.beep_tune/2;
//				__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,400);
//      }
//      else
//      {
//        LED_R_OFF;
//        g_err.beep_ctrl = 0;
//      }
//    }break;
    
    
    default:
    {
      LED_R_ON;
      g_err.beep_ctrl = 0;
    }break;
  }
}

static void module_fps_detect(void)
{
	for (int i = 0; i < MaxId; i++)
	{
		g_fps[i].fps = g_fps[i].cnt * 1000/DETECT_THREAD_PERIOD;
	}  
}

static void module_fps_clear(void)
{
	  for (int i = 0; i < MaxId; i++)
		{
			memset(&g_fps[i], 0, sizeof(global_fps_t));		
		}
}










