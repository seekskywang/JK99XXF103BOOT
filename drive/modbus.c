/******************** (C) COPYRIGHT 2019 AVER********************
 * 文件名  :MODBUS.C
 * 作者   :
 * 描述    :MODBUS?????
 * 内容    :KL220A_Master
 * 硬件连接: 485
 * 修改日期:2014-12-22
********************************************************************/
#include "my_register.h" 
#include "usart.h" 
#include "modbus.h" 
#include "stm32f10x.h"
#include "flash.h"
#include <string.h>
//===================================================================//
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
} flagA,flagB,flagC,flagD,flagE;
u8 testflag;
u8 dynaonflag;
uint32_t x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6;
CAL CalPara;
/*************************校准参数************************************/
vu8 DAC_Flag;//DAC是否加载标志
vu32 Modify_A_READ;
vu32 Modify_C_READ;
vu32 Modify_A_ACT;
	
vu32 Modify_B_READ;
vu32 Modify_D_READ;
vu32 Modify_B_ACT;

vu32 var16;
vu32 var32a;
vu32 var32b;

vu32 var16a;
vu32 var32c;
vu32 var32d;
				
vu16 Contr_DACVlue;//DAC加载值
vu32 Correct_Parametet[26];//校准参数
vu32 Correct_Strong[26];//校准系数
vu8 coefficient[13];
/*************************变量定义***********************************/
vu32 Run_Control[55];//运行数据寄存器
vu32 Transition_Date[10];//过渡变量
//============================函数声明===============================//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
//===========================MODBUS协议=============================//
void UART_Action(void)
{//RUT格式：
	//ADDR  命令码  读寄存器的起始地址高   读寄存器的起始地址低  读数据字个数高字节   读数据个数低字节  CRC高 CRC低
	//返回格式：ADDR 命令码 返回数据字节数  数据高  数据低 ..... CRC高  CRC低
	if ((UART_Buffer_Rece[0] == 0x01/*ADDR*/)||(UART_Buffer_Rece[0] == 0))
	{
		if (UART_Buffer_Rece[1] == (0x03))	//命令3 读数据   
		{																		 
			vu8 i;
			vu16 crc_result;
			crc_result = (UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];
			if ((crc_result == Hardware_CRC(UART_Buffer_Rece,6)) ||(crc_result == 0) )
			{
				if (UART_Buffer_Rece[3] < 0xFF)    								//如果寄存器在可读范围内
				{
					if ((UART_Buffer_Rece[3] + UART_Buffer_Rece[5]) < 0xFF)		//如果最后一个读取的寄存器地址在可读范围内
					{							
						UART_Buffer_Send[0] = 0x01/*ADDR*/;
						UART_Buffer_Send[1] = 0x03;
						UART_Buffer_Send[2] = UART_Buffer_Rece[5]*4;
						for (i=0;i<UART_Buffer_Send[2];i++)
						{
							if ((i % 4) == 0) 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 24;//高位在前
							}
							else if((i % 4) == 1)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 16;//高位在前
							}
							else if((i % 4) == 2)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 8;//高位在前
							}
							else 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4];			
							}															
						}
						crc_result = Hardware_CRC(UART_Buffer_Send,UART_Buffer_Send[2] + 3);
						UART_Buffer_Send[3 + UART_Buffer_Send[2]] = crc_result >> 8;
						UART_Buffer_Send[4 + UART_Buffer_Send[2]] = crc_result;
						Transmit_BUFFERsize = UART_Buffer_Send[2] + 5;
						UART_SEND_flag=1;
						UART2_Send();
					}
				}
			}	
		}
	} 
//===============================写寄存器=================================
	if ((UART_Buffer_Rece[0] == 0x01/*ADDR*/) || (UART_Buffer_Rece[0] == 0))	 
	{
		vu8 var8;
		vu8 a=0;
		vu16 var16;
		vu16 crc_result;
		vu32 var32;
//=========================以下命令6 写单个寄存器===========================
		if (UART_Buffer_Rece[1] == 0X06)                                 //判断第二个字节是否为命令6
		{
			if (UART_Buffer_Rece[3] < 0xFF)							  //判断需要写的地址是否在可写范围内
			{
				crc_result = (UART_Buffer_Rece[8] << 8) + UART_Buffer_Rece[9];
				if ((crc_result == Hardware_CRC(UART_Buffer_Rece,8)) ||(crc_result == 0) )		  //检查CRC
				{
					var32 = (UART_Buffer_Rece[4] << 8) + UART_Buffer_Rece[5];	//第5 6个字节为要写入的数据
					var16=(UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];	//第6  7个字节为要写入的数据
					var8 = UART_Buffer_Rece[3];	        						//第3 4个字节为要写入的地址
					var32=(var32<<16)+var16;
					Run_Control[var8] = var32;			    //将数据写入指定的地址

//					if (UART_Buffer_Rece[0] == ADDR)							//广播模式下不返回数据
//					{
						for (a=0;a<10;a++)
						{UART_Buffer_Send[a] = UART_Buffer_Rece[a];}
						Transmit_BUFFERsize = 10;						//原样数据返回，不计算CRC
						UART_SEND_flag=1;
						UART2_Send();
//					}
					Flag_Save_SW = 1;
//					Wite_Runcont();
					memset((char *)UART_Buffer_Rece,0,sizeof(UART_Buffer_Rece));
				}
			}
		}
//=======================================以下是命令16，连写寄存器===========================================
//功能码16格式:
//     地址 命令 写入起始地址高  写入起始地址低 写入字数高  写入字数低 写入寄存器个数  数据高 数据低 ......CRC高 CRC低
//返回数据格式:
//     地址 命令 写入起始地址高  写入起始地址低  写入字节数高 写入字节数低  CRC高  CRC低 
		if (UART_Buffer_Rece[1] == 0X10)										  
		{	
			if(MODE == 4 && DYNA_MODE == 1 && onoff_ch == 1)
			{
				dynaflagA = 0;
				dynaflagB = 1;
			}
			if(MODE == 4 && DYNA_MODE == 2 && onoff_ch == 1)
			{
				if(dynaflagA == 1)
				{
					dynaflagA = 0;
					dynaflagB = 1;
				}else if(dynaflagB == 1){
					dynaflagB = 0;
					dynaflagA = 1;
				}
			}
			crc_result = ((UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+7]) << 8) + UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+8];
			if (crc_result == Hardware_CRC(UART_Buffer_Rece,(UART_Buffer_Rece[6]*4)+7)) 	  //检查CRC
			{												
				for (var8=0;var8<UART_Buffer_Rece[6];var8++) 
				{
					var32 = (UART_Buffer_Rece[var8*4+7] << 8) + UART_Buffer_Rece[var8*4+8];
					var16=(UART_Buffer_Rece[var8*4+9] << 8) + UART_Buffer_Rece[var8*4+10];	//这里是写入的是32位寄存器所以是*4
					var32=(var32<<16)+var16;
					Run_Control[UART_Buffer_Rece[3]+var8] = var32;			    //将数据写入指定的地址
				}
				if(Run_Control[15] != 1)
				{
					testflag ++;
				}
				if (UART_Buffer_Rece[0] == 0x01/*ADDR*/)					  //广播模式不返回数据
				{
					UART_Buffer_Send[0] = 0x01/*ADDR*/;
					UART_Buffer_Send[1] = 16;
					UART_Buffer_Send[2] = UART_Buffer_Rece[2];
					UART_Buffer_Send[3] = UART_Buffer_Rece[3];
					UART_Buffer_Send[4] = UART_Buffer_Rece[4];
					UART_Buffer_Send[5] = UART_Buffer_Rece[5];
					crc_result = Hardware_CRC(UART_Buffer_Send,6);	 //计算CRC码
					UART_Buffer_Send[6] = crc_result>>8;
					UART_Buffer_Send[7] = crc_result;				 
					Transmit_BUFFERsize = 8;					     //设置发送字节数长度
					UART_SEND_flag=1;
					UART2_Send();
				}
				Flag_Save_SW = 1;
				if(MODE == 4 && onoff_ch == 1 && dynaonflag == 0)
				{
					dynaflagA = 1;
					dynaonflag = 1;
				}
//				Wite_Runcont();
				memset((char *)UART_Buffer_Rece,0,sizeof(UART_Buffer_Rece));
			}
		}			 
	}
/*************************************以下为校准部分**************************************************************************/

}




static float Bias_Para(u8 sw,u8 mode)
{
	float result;
	switch(mode)
	{
		case 0:result=(float)SET_Current;break;
		case 1:result=(float)SET_Voltage;break;
		case 2:result=(float)SET_R_Current;break;
		case 3:result=(float)SET_P_Current;break;	
	}
	if(sw)
		result/=10;
	return result;
}



//===============================AD值转换成测量值============================================//

//-----------------------------CRC检测--------------------------------------------//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count)    //CRC16
{
	vu16 CRC_Result=0xffff;
	vu8 reverse1;
	vu16 reverse2;
	vu8 i;
	if(count==0)
	{
		count=1;
	}
	while(count--)
	{
		CRC_Result^=*p_buffer;
		for(i=0;i<8;i++)
		{
			if(CRC_Result&1)
			{
				CRC_Result>>=1;
				CRC_Result^=0xA001;
			}
			else 
			{
				CRC_Result>>=1;
			}
		}
		p_buffer++;
	}
	return CRC_Result;
	
}

vu16 Hardware_CRC_Re(vu8 *p_buffer,vu8 count)    //CRC16
{
	vu16 CRC_Result=0xffff;
	vu8 reverse1;
	vu16 reverse2;
	vu8 i;
	if(count==0)
	{
		count=1;
	}
	while(count--)
	{
		CRC_Result^=*p_buffer;
		for(i=0;i<8;i++)
		{
			if(CRC_Result&1)
			{
				CRC_Result>>=1;
				CRC_Result^=0xA001;
			}
			else 
			{
				CRC_Result>>=1;
			}
		}
		p_buffer++;
	}
	//CRC高低位置换
	reverse1 = CRC_Result>>8;
	reverse2 = CRC_Result<<8;
	CRC_Result = reverse2+reverse1;
	return CRC_Result;
	
}