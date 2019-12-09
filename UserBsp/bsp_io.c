
/** @file bsp_io.c
 *  @version 4.0
 *  @date June 2019
 *
 *  @brief basic IO port operation
 *
 */

#include "bsp_io.h"
#include "cmsis_os.h" 
#define DURATION_PARAM    0.5f
#define TUNE_LEN              		6
/* Super Mario */
		int tune[]={1517,1517,1517,1912,1517,0,1276,0,2551,
		0,1912,0,2551,0,3030,0,2273,
		2024,2024,2273,2551,1517,1276,1136,0,1433,1276,
		0,1517,0,1912,1704,2024,0,1912,0,2551,0,
		0,3030,0,2273,2024,0,2024,2273,0,2551,1517,
		1276,1136,1433,1276,0,1517,1704,1912,2024,
		0,1276,1433,1433,1517,1517,2551,2273,1912,
		2273,1912,1704,0,1276,1433,1433,1517,0,1517,956,
		956,956,0,0,1276,1433,1433,1517,1517,
		0,2551,0,2273,1912,2273,1912,0,1704,0,1517,0,
		1704,0,1912,0,0,0,1276,
		0,1433,1433,1517,0,1517,0,2551,2273,1912,0,2273,1912,1704
		,0,1276,1433,1433,1517,0,1517,0,1912,0,1912,1912,
		0,0,1276,1433,1433,1517,0,1517,2551,2273,
		0,1912,0,2273,1912,1704,0,0,1517,0,0,1517,0,0,1704,0,
		1912,0,0,0,1912,1912,0,1912,0,
		1912,1704,0,1517,0,1912,0,2273,2551,0,1912,1912,
		0,1912,0,1912,1704,1517,0,0,
		1912,1912,0,1912,1912,1704,0,1517,1912,0,2273,2551,
		0,1517,1517,0,1517,0,1912,1517,0,1276,0,	
		0,2551,0,1912,0,2551,0,3030,
		0,2273,2024,2024,2273,2551,1517,1433,0,1136,
		0,2273,1276,0,1517,0,1912,1704,2024,0,1912,0,
		0,2273,1276,0,1517,0,1912,1704,2024,2273,
		2551,1517,1276,1136,0,1433,1276,0,1517,0,1912,1704,
		2024,0,1517,1912,0,2551,2551,2273,1443,
		0,1433,2273,0,2024,1136,1136,1136,1276,0,1433,
		1704,1912,2273,2551,0,0,1517,1912,0,2551,
		0,2551,2273,1433,0,1433,2273,0,2024,1433,
		0,1433,1433,1517,1704,0,1912,3030,0,3030,3817
		};
		int duration[] = {659,659,659,262,330,600,784,600,392,
		600,262,600,196,600,330,600,440,
		494,247,440,196,659,392,440,600,349,392,
		600,330,600,262,294,494,600,523,600,196,
		600,600,165,440,247,600,247,440,600,196,659,
		784,880,698,784,600,659,294,262,988,
		600,1568,349,349,659,659,196,440,523,600,
		220,262,587,600,392,349,698,659,600,659,1046,
		1046,600,600,392,349,349,659,330,
		600,196,600,264,523,220,262,600,294,600,659,600,
		587,600,523,600,600,600,784,
		600,349,349,330,600,330,600,196,220,523,600,220,262,294,
		600,392,349,349,659,600,330,600,262,600,523,523,	
		600,600,392,349,349,330,600,659,196,220,
		600,262,600,220,262,294,600,600,330,600,600,330,600,600,294,600,
		523,600,600,600,262,262,600,262,
		262,294,600,330,600,262,600,220,584,600,262,523,
		600,262,600,262,294,330,600,600,
		262,262,600,523,262,294,600,330,262,600,440,392,
		600,330,330,600,330,600,262,330,600,392,600,
		600,392,600,523,600,392,600,330,
		600,440,494,494,440,392,330,349,600,440,
		600,220,784,600,330,600,262,587,494,600,262,600,
		600,392,600,660,440,494,600,494,440,
		392,659,392,440,600,349,392,600,330,600,262,294,
		494,600,330,262,600,196,196,220,698,
		600,349,440,600,247,880,880,440,784,600,1396,
		587,523,440,392,600,600,330,262,600,392,
		600,196,220,349,600,349,880,600,247,698,
		600,349,698,330,294,600,262,330,600,165,262,600
		};

void GPIO_InitArgument(void)
{
   for (int i = 0; i <= 7; i++)
	{	
				DETECT_FLOW_LED_OFF(i);	
	}
  LASER_OFF;
	LED_R_OFF;
	LED_G_OFF;
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);	
//	busser_init(DURATION_PARAM,TUNE_LEN);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);	
}

void vcc_out_init(void)
{
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);  
}

uint16_t Length = sizeof(tune)/sizeof(tune[0]);
void busser_init(float arr, uint16_t length)
{
	static uint8_t void_flag;
  for(int i = 0; i < length; ++i)
	{
		if(void_flag)
		{
			HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
			void_flag = 0;
		}
		if(tune[i] == 0)
		{
			HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
			void_flag = 1;
		}
		else
		{
			TIM12_ARR = tune[i];
			__HAL_TIM_SET_AUTORELOAD(&htim12,TIM12_ARR);
		}
			__HAL_TIM_SET_COMPARE(&htim12 ,TIM_CHANNEL_1,TIM12_ARR/2);		
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);	  	
			HAL_Delay(arr*duration[i]);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);				
//			if ( i == length - 1)
//				i = 0;
	}
}

void flow_led_on(uint16_t num)
{	
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8 >> num,0);
}

void flow_led_off(uint16_t num)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8 >> num,1);
}

void flow_led(uint32_t period,uint8_t number)
{	
   for (int i = 0; i <= 7; i++)
   {

			 for (int j = 0; j < number; j++)
			 {
				  if(j!=2)
						DETECT_FLOW_LED_ON((i+j)%8);		 
			 }		
				osDelay((uint32_t)(period/8));
				for (int j = 0; j < number; j++)
			 {
				 	if(j!=2)
						DETECT_FLOW_LED_OFF((i+j)%8); 
			 }	 	 
		 
   }
}
