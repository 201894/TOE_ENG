
/** @file bsp_io.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief basic IO port operation
 *
 */

#include "bsp_io.h"
#include "cmsis_os.h"

void flow_led_on(uint16_t num)
{	
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8 >> num,0);
}

void flow_led_off(uint16_t num)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8 >> num,1);
}

void flow_led(void)
{	
	 
   for (int i = 0; i <= 8; i++)
   {
	   DETECT_FLOW_LED_ON(i);
	   osDelay(90);
	   DETECT_FLOW_LED_OFF(i);
   }
}