/******************** (C) COPYRIGHT 2015 AVER ********************
 * �ļ���  ��DAC.C
 * ����    ������
 * ����    ���⺯����
 * ����    ������16BIT DAC
 * Ӳ�����ӣ�
 * �޸����ڣ�2015-8-18
*********************************************************************/
#include "my_register.h" 
#include "dac.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/**
  * @brief  ʹ��DAC��ʱ�ӣ���ʼ��GPIO
  * @param  ��
  * @retval ��
  */
/********************************************************************/
#define DAC_SYNC_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_10);
#define DAC_SYNC_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_10);   
      
#define DAC_SCK_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_8);
#define DAC_SCK_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_8); 

#define DAC_SDA_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_9);
#define DAC_SDA_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_9); 
/**********************************************************************************************************
*	�� �� ��: AD5541_GPIOCoing
*	����˵��: ���ų�ʼ��
*	��    �Σ�
*	�� �� ֵ: ��
**********************************************************************************************************/
void AD5541_GPIOCoing(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//����GPIO�ṹ��
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_2MHz;//
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
	
}
/**********************************************************************************************************
*	�� �� ��: AD5541_GPIOCoing
*	����˵��: ���ų�ʼ��
*	��    �Σ�
*	�� �� ֵ: ��
**********************************************************************************************************/
void AD5541_Send(vu16 date)
{
	vu8 cont;
	DAC_SYNC_LOW;//SYNC�õ�
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	NOP;
	NOP;
	DAC_SYNC_HIGH;//SYNC�ø�
	NOP;
	DAC_SCK_HIGH;//SCK�ø�
	NOP;
	NOP;
	DAC_SYNC_LOW;//SYNC�õ�
	for(cont = 0; cont < 16; cont++)
	{
		if(date & 0x8000)		// �ȴ���λ
		{
			DAC_SDA_HIGH;
		}
		else
		{
			DAC_SDA_LOW;
		}
		DAC_SCK_LOW;//SCK�õ�
		NOP;
		NOP;
		DAC_SCK_HIGH;//SCK�ø�
		NOP;
		NOP;
		date=date<<1;
	}
	DAC_SYNC_HIGH;//SYNC�ø�
	DAC_SCK_HIGH;//SCK�ø�
}
