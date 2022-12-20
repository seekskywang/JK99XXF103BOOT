/******************************************************************/
/* 名称：main                                                    */
/* 效果：                                                        */
/* 内容：                                                       */
/* 作者：zhan                                                  */
/* 联系方式QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x.h"
#include "tim6.h"
#include "modbus.h"
#include "dac.h"
#include "AD7689.h"
#include "menu.h"
#include "adc.h"
#include "clock.h"
#include "gpio.h"
#include "bsp_SysTick.h"
#include "usart.h"
#include "iwdg.h"
#include "IIC_24C01.h"
#include "flash.h"
#include "beep.h"
#include "sys_io_cfg.h"
#include "me_scpi.h"
#include "FAN_PRO.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
#include "stmflash.h"
#include "iap.h"

struct bitDefine
{
    unsigned bit0: 1;
    unsigned bit1: 1;
    unsigned bit2: 1;
    unsigned bit3: 1;
    unsigned bit4: 1;
    unsigned bit5: 1;
    unsigned bit6: 1;
    unsigned bit7: 1;
} flagA, flagB,flagC,flagD,flagE,flagF,flagG;
vu16 date_dac;
RCC_ClocksTypeDef getrccclock;
u8 bootflag;
u8 iapfilebuf[IAP_FILE_SIZE];
u16 rxbtye;
u8 iaprespondbuf[10];
u32 iapwriteaddr;
u8 updaterecflag;
//收到升级数据包成功响应
void ResOK(void)
{
	vu16 crc;
	
	UART_Buffer_Send[0] = 0x01;
	UART_Buffer_Send[1] = 0x50;
	UART_Buffer_Send[2] = 0x00;
	UART_Buffer_Send[3] = 0x00;
	UART_Buffer_Send[4] = 0x00;
	UART_Buffer_Send[5] = 0x00;
	UART_Buffer_Send[6] = 0x00;
	UART_Buffer_Send[7] = 0x01;
	crc = Hardware_CRC(UART_Buffer_Send,8);
	UART_Buffer_Send[8] = crc >> 8;
	UART_Buffer_Send[9] = crc;
	Transmit_BUFFERsize = 10;
	UART_SEND_flag=1;
	UART2_Send();
}

//收到升级数据错误响应
void ResErr(void)
{
	vu16 crc;
	
	UART_Buffer_Send[0] = 0x01;
	UART_Buffer_Send[1] = 0x50;
	UART_Buffer_Send[2] = 0x00;
	UART_Buffer_Send[3] = 0x00;
	UART_Buffer_Send[4] = 0x00;
	UART_Buffer_Send[5] = 0x00;
	UART_Buffer_Send[6] = 0x00;
	UART_Buffer_Send[7] = 0xFF;
	crc = Hardware_CRC(UART_Buffer_Send,8);
	UART_Buffer_Send[8] = crc >> 8;
	UART_Buffer_Send[9] = crc;
	Transmit_BUFFERsize = 10;
	UART_SEND_flag=1;
	UART2_Send();
}

//升级完成响应
void ResFinish(void)
{
	vu16 crc;
	
	UART_Buffer_Send[0] = 0x01;
	UART_Buffer_Send[1] = 0x51;
	UART_Buffer_Send[2] = 0x00;
	UART_Buffer_Send[3] = 0x00;
	UART_Buffer_Send[4] = 0x00;
	UART_Buffer_Send[5] = 0x00;
	UART_Buffer_Send[6] = 0x00;
	UART_Buffer_Send[7] = 0x01;
	crc = Hardware_CRC(iaprespondbuf,8);
	UART_Buffer_Send[8] = crc >> 8;
	UART_Buffer_Send[9] = crc;
	Transmit_BUFFERsize = 10;
	UART_SEND_flag=1;
	UART2_Send();
}

int main(void)
{
	u16 oldcount=0;				//老的串口接收数据值
	static u16 crc;
	__enable_irq();
	RCC_Configuration();
	SysTick_Init();
	GPIO_Conf();
//	AD5541_GPIOCoing();
//	AD7689_InitializeSPI2();

//	ADC1_DMA_Init();
	i2c_CfgGpio();
	
	EEPROM_READ_Coeff();//读取校准参数
//	USART_Configuration();
	USART2_Configuration();
//	USART3_Configuration();
//	TIM6_Config();
//	IWDG_Inte();
	Read_bootflag();
//	Read_bootmode();
	
	if(BOOTMODE == 0)
	{
		if(bootflag==0)
		{
			if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
			{	 
				__disable_irq();
				iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
			}
		}else{
			bootflag=0; 
			Write_bootflag();
		}
	}
	FILESIZE=0;
	iapwriteaddr=FLASH_APP1_ADDR;
	while(1)
	{
//		RCC_GetClocksFreq(&getrccclock);
//		IWDG_ReloadCounter(); //喂狗
		if(USART_RX_CNT)
		{
			if(oldcount==USART_RX_CNT)//新周期内,没有收到任何数据,认为本次数据接收完成.
			{
				
//					applenth=USART_RX_CNT;
				if(recupfileflag == 1)
				{
					crc = Hardware_CRC(uprecbuf,USART_RX_CNT-2);
					if(crc == (uprecbuf[USART_RX_CNT-2]<<8) + uprecbuf[USART_RX_CNT-1])
					{
						memcpy(iapfilebuf,uprecbuf,USART_RX_CNT-2);
						rxbtye+=USART_RX_CNT-2;//接收数据计数
						iap_write_appbin(iapwriteaddr,iapfilebuf,USART_RX_CNT-2);//写入升级分包
						iapwriteaddr+=USART_RX_CNT-2;//写入地址偏移
						ResOK();
						oldcount=0;
						USART_RX_CNT=0;
					}else{//升级包校验错误，上电强制BOOT
//						BOOTMODE=1;
//						Write_bootmode();
//						ResErr();
					}
					if(rxbtye==FILESIZE && FILESIZE!=0)
					{
						Delay_us(100);
						rxbtye=0;
						ResFinish();
						__disable_irq();
						iap_load_app(FLASH_APP1_ADDR);
		//				applenth=rxbtye
//						File_IAP();
					}
				}
//				else{
//					crc = CRC16(USART_RX_BUF,USART_RX_CNT-2);
//					if(crc == (USART_RX_BUF[USART_RX_CNT-2]<<8) + USART_RX_BUF[USART_RX_CNT-1])
//					{
//						USART_RX_CNT=0;
//						if(USART_RX_BUF[0] == 'S' && USART_RX_BUF[1] == 'I' && USART_RX_BUF[2] == 'Z' && USART_RX_BUF[3] == 'E')
//						{
//							applenth=((uint32_t)USART_RX_BUF[4]<<24)+((uint32_t)USART_RX_BUF[5]<<16)+((uint32_t)+USART_RX_BUF[6]<<8)+((uint32_t)USART_RX_BUF[7]);
//							rxflag=1;
//							ResSIZE();
//						}
//					}
//				}
			}else oldcount=USART_RX_CNT;			
		}
//		if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
//		{	 
//			iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
//		}else 
//		{
////					   
//		}
//		if(UART_Buffer_Rece_flag==1)
//		{
//			UART_Buffer_Rece_flag=0;
//			UART_Action();//接收一帧数据
//			Baud_SET();//设置串口波特率
////			MAXPAR_limit();//运行参数最大值限制
//		}
//		if(UART1_Buffer_Rece_flag==1) 
//		{
//			UART1_Buffer_Rece_flag=0;
//			UART1_Action();//接收一帧数据
////			Baud_SET();//设置串口波特率
////			MAXPAR_limit();//运行参数最大值限制
//		}
//		Me_SCPI_TASK(); //SCPI串口任务
//		Wite_Runcont();//将运行参数写入EEPROM
//		AD5541_Send(Contr_DACVlue);//设置DAC值
//		AD7689_Scan_CH();//读取AD7689 AD值
//		if(DAC_Flag == 0x00)
//		{		
//			Transformation_ADC(); 
//			if(SWDelay == 0)
//			{
//				All_protect();//各种保护	
//			}else if(SWDelay > 0){
//				SWDelay--;
//			}
//		}
//		if(Flag_ADC_Full==1) //NTC
//		{
//			Flag_ADC_Full=0;
//			ADC_CH_Scan();
//		} 
//		Temp_Comapre();//风扇
//		worke_mode();//工作模式切换
	}
}


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
