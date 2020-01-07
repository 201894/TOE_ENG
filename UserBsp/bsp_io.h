/** @file bsp_io.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief basic IO port operation
 *
 *  HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
 */
#ifndef __BSP_IO_H__
#define __BSP_IO_H__
#include "stm32f4xx_hal.h"
#include  "gpio.h"
#include "tim.h"

#define LASER_ON    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define LASER_OFF   HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)
#define LED_G_ON   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_G_OFF  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_G_TOG  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14)
#define LED_R_ON   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_R_OFF  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET)
#define LED_R_TOG  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11)

#define led_init \
{\
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);\
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);\
}\

#define DETECT_FLOW_LED_ON(i) flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i)
// EVALVE REVELENT
#define EVALVE_GPIO_PORT   	GPIOH
#define CLAMP_PIN											GPIO_PIN_2		
#define LAUNCH_PIN 									GPIO_PIN_3
#define DEFORM_PIN									GPIO_PIN_4
			/*		SparePin £ºPH5			*/
void busserCtrl(float arr, uint16_t length);
void oledCtrl(void);
void vcc_out_init(void);
void GPIO_InitArgument(void);
void ledFlow(uint32_t period,uint8_t number);
void flow_led_on(uint16_t num);
void flow_led_off(uint16_t num);
void flow_led_tog(uint16_t num);
#endif
