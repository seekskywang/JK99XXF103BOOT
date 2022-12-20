/******************** (C) COPYRIGHT 2019 AVER********************
 * �ļ���  :MODBUS.C
 * ����   :
 * ����    :MODBUS?????
 * ����    :KL220A_Master
 * Ӳ������: 485
 * �޸�����:2014-12-22
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
/*************************У׼����************************************/
vu8 DAC_Flag;//DAC�Ƿ���ر�־
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
				
vu16 Contr_DACVlue;//DAC����ֵ
vu32 Correct_Parametet[26];//У׼����
vu32 Correct_Strong[26];//У׼ϵ��
vu8 coefficient[13];
/*************************��������***********************************/
vu32 Run_Control[55];//�������ݼĴ���
vu32 Transition_Date[10];//���ɱ���
//============================��������===============================//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
//===========================MODBUSЭ��=============================//
void UART_Action(void)
{//RUT��ʽ��
	//ADDR  ������  ���Ĵ�������ʼ��ַ��   ���Ĵ�������ʼ��ַ��  �������ָ������ֽ�   �����ݸ������ֽ�  CRC�� CRC��
	//���ظ�ʽ��ADDR ������ ���������ֽ���  ���ݸ�  ���ݵ� ..... CRC��  CRC��
	if ((UART_Buffer_Rece[0] == 0x01/*ADDR*/)||(UART_Buffer_Rece[0] == 0))
	{
		if (UART_Buffer_Rece[1] == (0x03))	//����3 ������   
		{																		 
			vu8 i;
			vu16 crc_result;
			crc_result = (UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];
			if ((crc_result == Hardware_CRC(UART_Buffer_Rece,6)) ||(crc_result == 0) )
			{
				if (UART_Buffer_Rece[3] < 0xFF)    								//����Ĵ����ڿɶ���Χ��
				{
					if ((UART_Buffer_Rece[3] + UART_Buffer_Rece[5]) < 0xFF)		//������һ����ȡ�ļĴ�����ַ�ڿɶ���Χ��
					{							
						UART_Buffer_Send[0] = 0x01/*ADDR*/;
						UART_Buffer_Send[1] = 0x03;
						UART_Buffer_Send[2] = UART_Buffer_Rece[5]*4;
						for (i=0;i<UART_Buffer_Send[2];i++)
						{
							if ((i % 4) == 0) 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 24;//��λ��ǰ
							}
							else if((i % 4) == 1)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 16;//��λ��ǰ
							}
							else if((i % 4) == 2)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 8;//��λ��ǰ
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
//===============================д�Ĵ���=================================
	if ((UART_Buffer_Rece[0] == 0x01/*ADDR*/) || (UART_Buffer_Rece[0] == 0))	 
	{
		vu8 var8;
		vu8 a=0;
		vu16 var16;
		vu16 crc_result;
		vu32 var32;
//=========================��������6 д�����Ĵ���===========================
		if (UART_Buffer_Rece[1] == 0X06)                                 //�жϵڶ����ֽ��Ƿ�Ϊ����6
		{
			if (UART_Buffer_Rece[3] < 0xFF)							  //�ж���Ҫд�ĵ�ַ�Ƿ��ڿ�д��Χ��
			{
				crc_result = (UART_Buffer_Rece[8] << 8) + UART_Buffer_Rece[9];
				if ((crc_result == Hardware_CRC(UART_Buffer_Rece,8)) ||(crc_result == 0) )		  //���CRC
				{
					var32 = (UART_Buffer_Rece[4] << 8) + UART_Buffer_Rece[5];	//��5 6���ֽ�ΪҪд�������
					var16=(UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];	//��6  7���ֽ�ΪҪд�������
					var8 = UART_Buffer_Rece[3];	        						//��3 4���ֽ�ΪҪд��ĵ�ַ
					var32=(var32<<16)+var16;
					Run_Control[var8] = var32;			    //������д��ָ���ĵ�ַ

//					if (UART_Buffer_Rece[0] == ADDR)							//�㲥ģʽ�²���������
//					{
						for (a=0;a<10;a++)
						{UART_Buffer_Send[a] = UART_Buffer_Rece[a];}
						Transmit_BUFFERsize = 10;						//ԭ�����ݷ��أ�������CRC
						UART_SEND_flag=1;
						UART2_Send();
//					}
					Flag_Save_SW = 1;
//					Wite_Runcont();
					memset((char *)UART_Buffer_Rece,0,sizeof(UART_Buffer_Rece));
				}
			}
		}
//=======================================����������16����д�Ĵ���===========================================
//������16��ʽ:
//     ��ַ ���� д����ʼ��ַ��  д����ʼ��ַ�� д��������  д�������� д��Ĵ�������  ���ݸ� ���ݵ� ......CRC�� CRC��
//�������ݸ�ʽ:
//     ��ַ ���� д����ʼ��ַ��  д����ʼ��ַ��  д���ֽ����� д���ֽ�����  CRC��  CRC�� 
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
			if (crc_result == Hardware_CRC(UART_Buffer_Rece,(UART_Buffer_Rece[6]*4)+7)) 	  //���CRC
			{												
				for (var8=0;var8<UART_Buffer_Rece[6];var8++) 
				{
					var32 = (UART_Buffer_Rece[var8*4+7] << 8) + UART_Buffer_Rece[var8*4+8];
					var16=(UART_Buffer_Rece[var8*4+9] << 8) + UART_Buffer_Rece[var8*4+10];	//������д�����32λ�Ĵ���������*4
					var32=(var32<<16)+var16;
					Run_Control[UART_Buffer_Rece[3]+var8] = var32;			    //������д��ָ���ĵ�ַ
				}
				if(Run_Control[15] != 1)
				{
					testflag ++;
				}
				if (UART_Buffer_Rece[0] == 0x01/*ADDR*/)					  //�㲥ģʽ����������
				{
					UART_Buffer_Send[0] = 0x01/*ADDR*/;
					UART_Buffer_Send[1] = 16;
					UART_Buffer_Send[2] = UART_Buffer_Rece[2];
					UART_Buffer_Send[3] = UART_Buffer_Rece[3];
					UART_Buffer_Send[4] = UART_Buffer_Rece[4];
					UART_Buffer_Send[5] = UART_Buffer_Rece[5];
					crc_result = Hardware_CRC(UART_Buffer_Send,6);	 //����CRC��
					UART_Buffer_Send[6] = crc_result>>8;
					UART_Buffer_Send[7] = crc_result;				 
					Transmit_BUFFERsize = 8;					     //���÷����ֽ�������
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
/*************************************����ΪУ׼����**************************************************************************/

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



//===============================ADֵת���ɲ���ֵ============================================//

//-----------------------------CRC���--------------------------------------------//
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
	//CRC�ߵ�λ�û�
	reverse1 = CRC_Result>>8;
	reverse2 = CRC_Result<<8;
	CRC_Result = reverse2+reverse1;
	return CRC_Result;
	
}