/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * ����    :usart.h
 * ����    :����
 * ����    :
 * �������:
 * Ӳ������:
 * �޸�����:2015-2-5
*******************************************************************/
	

#ifndef _usart_h_
#define _usart_h_

#include "stm32f10x.h"

void USART_Configuration(void);//����1��ʼ������
void USART2_Configuration(void);//����2��ʼ������
void USART3_Configuration(void);//����3��ʼ������
void UART1_Send(void);
void UART2_Send(void);
void UART3_Send(void);
vu8 Usart1_Sendstring(char *s );
static void USART1_NVIC_Config(void);
void Clear_Date(vu8 *sd , vu8 length);
void Baud_SET(void);
#endif
	