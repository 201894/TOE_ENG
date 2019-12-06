
/** @file bsp_io.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief basic IO port operation
 *
 */

#include "bsp_io.h"
#include "cmsis_os.h"

void vcc_out_init(void)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);  
}

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
	 
   for (int i = 0; i <= 7; i++)
   {
	   DETECT_FLOW_LED_ON(i);
	   DETECT_FLOW_LED_ON((i+3)%8);	
	   DETECT_FLOW_LED_ON((i+2)%8);	
	   DETECT_FLOW_LED_ON((i+1)%8);			 
	   osDelay(80);
	   DETECT_FLOW_LED_OFF(i);
	   DETECT_FLOW_LED_OFF((i+3)%8);
	   DETECT_FLOW_LED_OFF((i+2)%8);	
	   DETECT_FLOW_LED_OFF((i+1)%8);			 
   }
}
