/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "my_register.h"
#include "AD7689.h"
#include "modbus.h"
#include "usart.h"
#include "flash.h"
#include <string.h>
//#include "scpi/scpi.h"
//#include "scpi-def.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
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
} flagA,flagB,flagC,flagD,flagE,flagF;
vu8 UART1_Buffer_Rece_flag;
vu8 UART3_Buffer_Rece_flag;
vu8 dynaflagA;
vu8 dynaflagB;
vu32 timecount;
vu32 testcount;
vu8 dynatrigflag;
vu8 recupfileflag=0;
u8 uprecbuf[1026];
u16 USART_RX_CNT;
extern __IO int32_t OS_TimeMS;
extern u8 updaterecflag;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
	if(flag_Tim_USART==1)//�����������
	{
		t_USART++;
	}
	if(t_USART>250)//��Լ2.6ms
	{
		t_USART=0;
		flag_Tim_USART=0;
		UART_Buffer_Size=0;	
	}
}

void USART1_IRQHandler(void)//232
{
	uint8_t Res,scpires;
	flag_Tim_USART=1;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit( USART1, USART_IT_RXNE );
		
		if(COMM_MODE == 0)
		{
//			UART_Buffer_Rece1[UART_Buffer_Size]=USART_ReceiveData(USART1);
//			if( UART_Buffer_Rece1[0]== ADDR)
//			{
//				Operation_MODE = 1;
//				if(UART_Buffer_Rece1[1]== 0x06)
//				{
//					if(UART_Buffer_Size>8)//���ò���
//					{
//						UART_Buffer_Size=0;	  	   		   
//						UART1_Buffer_Rece_flag=1;
//						flag_Tim_USART=0;
//						t_USART=0;
//						if(UART1_Buffer_Rece_flag==1)
//						{
//							UART1_Buffer_Rece_flag=0;
//							UART1_Action();//��������
//							Write_ADDR();
//						}
//						return ;
//					}
//				}else if(UART_Buffer_Rece1[1]== 0x10){
//					if(UART_Buffer_Size > 7 && 
//						 UART_Buffer_Size == UART_Buffer_Rece1[5]+9-1)//��������
//					{
//						UART_Buffer_Size=0;	  	   		   
//						UART1_Buffer_Rece_flag=1;
//						flag_Tim_USART=0;
//						t_USART=0;
//						if(UART1_Buffer_Rece_flag==1)
//						{
//							UART1_Buffer_Rece_flag=0;
//							UART1_Action();//��������
//							Write_ADDR();
//						}
//						return ;
//					}
//				}else if(UART_Buffer_Rece1[1]== 0x03){
//					if(UART_Buffer_Size==7)//������
//					{
//						UART_Buffer_Size=0;	  	   		   
//						UART1_Buffer_Rece_flag=1;
//						flag_Tim_USART=0;
//						t_USART=0;
//						if(UART1_Buffer_Rece_flag==1)
//						{
//							UART1_Buffer_Rece_flag=0;
//							UART1_Action();//��������
//						}
//						return ;
//					}
//				}else if(UART_Buffer_Rece1[1]== 0xA5){
//					if(UART_Buffer_Size==8)//У׼
//					{
//						UART_Buffer_Size=0;	  	   		   
//						UART1_Buffer_Rece_flag=1;
//						flag_Tim_USART=0;
//						t_USART=0;
//						if(UART1_Buffer_Rece_flag==1)
//						{
//							UART1_Buffer_Rece_flag=0;
//							UART1_Action();//��������
//						}
//						return ;
//					}
//				}
//			}else if( UART_Buffer_Rece1[0]== 0x00)//д���ַ
//			{
//				if(UART_Buffer_Size>8)//��ַ
//				{
//					UART_Buffer_Size=0;	  	   		   
//					UART1_Buffer_Rece_flag=1;
//					flag_Tim_USART=0;
//					t_USART=0;
//					if(UART1_Buffer_Rece_flag==1)
//					{
//						UART1_Buffer_Rece_flag=0;
//						UART1_Action();//��������
//					}
//					return ;
//				}
//			}else{
//				memset((char *)UART_Buffer_Rece1,0,sizeof(UART_Buffer_Rece1));
//				UART_Buffer_Size=0;
//				UART1_Buffer_Rece_flag=0;
//				t_USART=0;
//			}
//	//		else if(UART_Buffer_Rece[UART_Buffer_Size] == 0x0A) //scpiָ���� �ж�βָ���Ƿ�Ϊ\n
//	//		{
//	//			flag_NOR_CODE=1;
//	//			UART_Buffer_Size=0;
//	//			flag_Tim_USART=0;				
//	//			t_USART=0;	
//	//			return ;
//	//		}
//			

//			UART_Buffer_Size++;
//			if(UART_Buffer_Size>200)
//			{
//				UART_Buffer_Size=0;
//				UART1_Buffer_Rece_flag=0;
//				t_USART=0;	
//			}
		}else if(COMM_MODE == 2){//SCPIģʽ
			
//			Res=USART_ReceiveData(USART1);	
//			if(SCPI_Input(&scpi_context, &Res, 1))
//			{
//				Operation_MODE = 1;
//			}
		}else{
//			UART_Buffer_Rece1[UART_Buffer_Size]=USART_ReceiveData(USART1);
//			UART_Buffer_Size=0;
//			UART1_Buffer_Rece_flag=0;
//			t_USART=0;	
		}
		
	}
}

void USART2_IRQHandler(void)
{
	flag_Tim_USART=1;
	u8 dat;
//	
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//	{
//		if(recupfileflag == 0)
//		{
//			UART_Buffer_Rece[UART_Buffer_Size++]=USART_ReceiveData(USART2);
//		}else if(recupfileflag == 1){
//			dat=USART_ReceiveData(USART2);
//	    if(USART_RX_CNT<1026)
//	    {
//	      uprecbuf[USART_RX_CNT]=dat;
//				USART_RX_CNT++;			 									     
//	    }
//		}
//	}else if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)//���յ�һ֡���� �����жϴ���
//	{
//		USART3->SR;//�ȶ�SR
//		USART3->DR;//�ٶ�DR
//		if(recupfileflag == 0)
//		{
//			UART_Buffer_Rece_flag=1;//������ɱ�־λ
//			UART_Buffer_Size=0;
//			flag_Tim_USART=0;
//			t_USART=0;
//			if(UART_Buffer_Rece_flag==1)
//			{
//				UART_Buffer_Rece_flag=0;
//				UART_Action();//��������
//				if(FILESIZE!=0)
//					recupfileflag=1;
//			}
//		}else if(recupfileflag == 1){
//			updaterecflag=1;
//		}
//	}
	
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit( USART2, USART_IT_RXNE );		
		if(recupfileflag == 0)
		{
			UART_Buffer_Rece[UART_Buffer_Size]=USART_ReceiveData(USART2);
			if( UART_Buffer_Rece[0]== 0x01)
			{
				if(UART_Buffer_Rece[1]== 0x06)
				{
					if(UART_Buffer_Size>8)//���ò���
					{
						UART_Buffer_Size=0;	  	   		   
						UART_Buffer_Rece_flag=1;
						flag_Tim_USART=0;
						t_USART=0;
						if(UART_Buffer_Rece_flag==1)
						{
							UART_Buffer_Rece_flag=0;
							UART_Action();//��������
							if(FILESIZE!=0)
								recupfileflag=1;
						}
						return ;
					}
				}
			}
			UART_Buffer_Size++;
			if(UART_Buffer_Size>200)
			{
				UART_Buffer_Size=0;
				UART_Buffer_Rece_flag=0;
				t_USART=0;	
			}
		}else if(recupfileflag == 1){
		  dat=USART_ReceiveData(USART2);
	    if(USART_RX_CNT<1026)
	    {
	      uprecbuf[USART_RX_CNT]=dat;
				USART_RX_CNT++;			 									     
	    }
		}
		
	}
}



void DMA1_Channel1_IRQHandler(void)
{
	Flag_ADC_Full=1;//һ��ͨ���ɼ���ɱ�־
	DMA_ClearITPendingBit(DMA1_IT_TC1);//����жϱ�־λ
}

void DMA1_Channel2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC2);//����жϱ�־λ
	Flag_AD7655_DMA=1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
