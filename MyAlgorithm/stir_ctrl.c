
/** @file __STIR_CTRL_C__
 *  @version 1.0
 *  @date  2020.1.12
 *
 *  @brief kernal logic mode switch ,and many important info handle
 *
 *
 */
 
#include "kernal_thread.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "pid.h"
#include "math.h"
#include "STMGood.h"
#include "km_handle.h"
#include "gimbal_thread.h"
#define ENERGY_ADD     10
// ��������
int shoot_once(void)
{
	static uint32_t count = 0;
	if(fabs(kernal_ctrl.stirTargetAngle - moto_gimbal[MotoStir].total_angle ) < 80.9){//81.0����ȼӵ�ֵ81.91С
		kernal_ctrl.stirTargetAngle += 81.910f;
		count = 0;
		return 1;
	}
	else
		count ++;
	return 0;
}
// ��������
int is_continue_shoot(bool is_shoot)
{
	static int count = 0;
	count++;
	if(!is_shoot)   return 0;
	if(is_shoot && (count*KERNAL_THREAD_PERIOD > (int)(1000.0f/kernal_ctrl.shootFreq)) && (kernal_ctrl.shootFreq != 0))
	{
			if(fabs(pid_fric[FricLeft].errNow) < 1000.0f && fabs(pid_fric[FricRight].errNow) < 1000.0f){
				int is_shoot_over = shoot_once();
				count = 0;
				if(is_shoot_over) return 1;
				return 0;
			}
	}
	return 0;
}
// ��Ƶ����
int shoot_freq_ctrl(bool is_shoot,uint16_t *energy)
{
	static int energyuse = 0;
	if(is_shoot == 0){
		kernal_ctrl.shootFreq = 0;
		return 0;
	}
	int xenergy = (int)*energy;	
	energyuse = 200 - xenergy;			
}



