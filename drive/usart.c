/******************************************************************/
/* ���ƣHuasrt                                           */
/* Ч����                                                        */
/* ���ݣ�                                                        */
/* ���ߣ�����                                                    */
/* ��ϵ��ʽ��QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "string.h"
#include "stdio.h"
#include "string.h"

/*****************************************************************/
extern struct bitDefine
{
	unsigned bit0: 1;
	unsigned bit1: 1;
	unsigned bit2: 1;
	unsigned bit3: 1;
	unsigned bit4: 1;
	unsigned bit5: 1;
	unsigned bit6: 1;
	unsigned bit7: 1;
} flagA,flagB,flagC,flagD;
vu8 UART_Buffer_Rece[200];
vu8 UART_Buffer_Send[200];
vu8 UART_Buffer_Rece1[200];
vu8 UART_Buffer_Send1[200];
vu8 UART_Buffer_Size;
vu8 Transmit_BUFFERsize;
vu8 t_USART;
vu32 Baud=9600;
/*****************************************************************/
static void USART1_NVIC_Config(void)//���ڽ����ж�����
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	

	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

	
}



void USART2_Configuration(void)//���ڳ�ʼ������
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.03) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART1_NVIC_Config();//�����ж�����
	
	/******ʹ�ܴ��ڽ����ж�******************/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //�����ж�	
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //�����ж�
	USART_Cmd(USART2, ENABLE);//ʹ�ܴ���2
    USART_ClearFlag(USART2, USART_FLAG_TC);
  
}






void UART2_Send(void)
{
	vu8 i=0;
	for(i =0 ; i < Transmit_BUFFERsize; i++)
  {
			USART_SendData(USART2,UART_Buffer_Send[i]);
			while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);//�ȴ��������
  }
	UART_SEND_flag=0;//�������һ֡����		
}

/**********************************************************************************************************
*	�� �� ��: Baud_SET
*	����˵��: ���ڲ������޸�
*	��    ��:  ��
*	�� �� ֵ: ��
**********************************************************************************************************/


/*******************************************************************************
* Function Name  : DMA_Configuration					�������ƣ�ʹ��USART1����һ���ַ�
* Description    : Configures the DMA.				����		��
* Input          : None												����		����
* Output         : None												���		����
* Return         : None												����		: ��
*******************************************************************************/ 
vu8 Usart1_Sendstring(char *s )
{
	vu8 i;
	vu8 length;
	length = strlen(s);
	
	for( i=0; i<length ; i++ ) //Ϊ�ַ��������� '\0'���˳�
	{		
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  
		USART_SendData(USART1, s[i]); 
	}
	return 1;
}

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//////////////////////////////////////////////////////////////
void Clear_Date(vu8 *sd , vu8 length)
{
	vu8 i;
	for( i=0; i<length ; i++ ) //�������
	{		
		sd[i]=0;
	}
}
