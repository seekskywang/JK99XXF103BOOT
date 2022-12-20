/******************************************************************/
/* ����:�Hiwdg.c                                                  */
/* Ч����                                                        */
/* ���ݣ� ���Ź���ʼ��                                                       */
/* ���ߣ�����                                                    */
/* ��ϵ��ʽ��QQ:363116119                                        */
/******************************************************************/
#include "stm32f10x_iwdg.h"

/*****************************************************/
void IWDG_Inte(void)
{
	/* �����ص���������ֵ ���250ms��IWDG��ʱ���԰�������ʽ���㣺  
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (0xff/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(0xff);
  IWDG_ReloadCounter();
	IWDG_Enable();
}
