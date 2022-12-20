/******************** (C) COPYRIGHT 2019 AVER********************
 * �ļ���  :MENU.C
 * ����   :
 * ����    :
 * ����    :
 * Ӳ������: 
 * �޸�����:
********************************************************************/
#include "my_register.h" 
#include "usart.h" 
#include "modbus.h" 
#include "stm32f10x.h"
#include "flash.h"
#include "menu.h"
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
vu8 Von_CONT;//���ؿ����ż����μ�����
vu8 oldmode;
vu16 SWDelay;
/***************************************
������:Sence_SW_CONT
��������:
��������:Զ�˲�������
****************************************/
void Sence_SW_CONT(void)
{
	if(Sence_SW==1)
	{
//		GPIO_SetBits(GPIOA,GPIO_Pin_6);//����Զ�˲���
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);//����Զ�˲���
	}
	else 
	{
//		GPIO_ResetBits(GPIOA,GPIO_Pin_6);//�ر�Զ�˲���
		GPIO_SetBits(GPIOA,GPIO_Pin_6);//�ر�Զ�˲���
	}
}
/***************************************
������:I_SW_COTNR
��������:
��������:������λ�л�����
****************************************/
void I_SW_COTNR(void)
{
	if(I_Gear_SW==1)
	{
//		GPIO_SetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�͵�λ
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
	}
	else
	{
//		GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
		GPIO_SetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�͵�λ
	}
	if(DAC_Flag == 0)
	{
		if(I_Gear_SW==0)
		{
			if(Current>I_LOW_MAX)//�������������ڵ͵���ߵ���ʱ�Զ��л��ɸߵ�λ�����������ֶ��л����ܱ�ɵ͵�λ
			{
				GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
				I_Gear_SW=1;//�Զ��л�Ϊ�ߵ�λ
				SWDelay = SWDELAY;
			}
			if(MODE == 0)//CC
			{
				if(SET_Current>I_LOW_MAX)//�������������ڵ͵���ߵ���ʱ�Զ��л��ɸߵ�λ�����������ֶ��л����ܱ�ɵ͵�λ
				{
					GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
					I_Gear_SW=1;//�Զ��л�Ϊ�ߵ�λ
					SWDelay = SWDELAY;
				}
			}else if(MODE == 2){//CR
				if(SET_R_Current>I_LOW_MAX)//�������������ڵ͵���ߵ���ʱ�Զ��л��ɸߵ�λ�����������ֶ��л����ܱ�ɵ͵�λ
				{
					GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
					I_Gear_SW=1;//�Զ��л�Ϊ�ߵ�λ
					SWDelay = SWDELAY;
				}
			}else if(MODE == 3){//CP
				if(SET_P_Current>I_LOW_MAX)//�������������ڵ͵���ߵ���ʱ�Զ��л��ɸߵ�λ�����������ֶ��л����ܱ�ɵ͵�λ
				{
					GPIO_ResetBits(GPIOB,GPIO_Pin_1);//��������Ϊ�ߵ�λ
					I_Gear_SW=1;//�Զ��л�Ϊ�ߵ�λ
					SWDelay = SWDELAY;
				}
			}
		}
	}
}
/***************************************
������:V_SW_COTNR
��������:
��������:��ѹ��λ�л�
****************************************/
void V_SW_COTNR(void)
{
	if(V_Gear_SW==0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_11);//��ѹ��λΪ�͵� 0-15V
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_11);//��ѹ��λΪ�ߵ� 0-150V
	}
	if(DAC_Flag == 0)
	{
		if(V_Gear_SW==0)
		{
			if(Voltage>V_LOW_MAX)//��������ѹ���ڵ͵�������Ƶ�ѹ�ǵ�λ�Զ���ת���ߵ�
			{
				V_Gear_SW=1;
				GPIO_ResetBits(GPIOA,GPIO_Pin_11);//��ѹ��λΪ�ߵ�
			}
		}
	}
}
/***************************************
������:worke_mode
��������:
��������:����ģʽ�л�
****************************************/
void worke_mode(void)
{
/**********ģʽ�л�**********************/
	if(MODE==0)
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
			SWDelay = SWDELAY;
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CCģʽ
	}
	else if(MODE==1)
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
			SWDelay = SWDELAY;
		}
		GPIO_SetBits(GPIOB,GPIO_Pin_0);//CVģʽ
	}
	else if(MODE==2)
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
			SWDelay = SWDELAY;
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CRģʽ
//		if(SET_Resist==0)
//		{
//			SET_Resist=1;
//		}
	}
	else if(MODE==3)
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
			SWDelay = SWDELAY;
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CPģʽ
	}
	else if(MODE==4)//��̬ģʽ
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CCģʽ����
	}
	else if(MODE==5)//LEDģʽ LEDģʽ��ѹ��λĬ���л����ߵ�λ
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CCģʽ
		V_Gear_SW=1;
		GPIO_SetBits(GPIOA,GPIO_Pin_11);//��ѹ��λΪ�ߵ�
	}
	else if(MODE==6)//��·ģʽ
	{
		if(oldmode != MODE)
		{
			TIME_1MS_OVER=0;//��������־
			TIME_1MS_flag=0;//����ʱ���־
			SWDelay = SWDELAY;
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CCģʽ����
		if(I_Gear_SW==0)
		{
			SET_S_Current=199000;
		}
		else
		{
			SET_S_Current=180000;//��·ģʽ��ֱ�Ӽ���������ֵ
		}
	}
	oldmode = MODE;
/*************ON/OFF����***************************/
	if(onoff_ch==0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
		TIME_1MS_OVER=0;//��������־
		TIME_1MS_flag=0;//����ʱ���־
		SET_I_TRAN=0;
		Von_CONT=0;//��������ż�������
		SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
		dynaflagA = 0;
		dynaflagB = 0;
		dynaonflag = 0;
	}
	else if(onoff_ch==1)
	{
		if((MODE==0)||(MODE==1)||(MODE==2)||(MODE==3))
		{
			if(V_Gear_SW==1)//��ѹ�ߵ�λ
			{
				if(Voltage<VOFF_Voltage)//�жϲ�����ѹ�Ƿ�С��ж�ص�ѹ
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
				}
				if((Voltage>VON_Voltage)&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
					Von_CONT=1;
				}
				else if((Voltage<VON_Voltage)&&(Von_CONT==0))
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					Von_CONT=0;//��������ż�������
				}
			}
			else if(V_Gear_SW==0)//��ѹ�͵�λVON VOFF��*10����Ϊ�趨����ֻ֧�ָߵ�λ�趨
			{
				if(Voltage<(VOFF_Voltage*10))//�жϲ�����ѹ�Ƿ�С��ж�ص�ѹ
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
				}
				if((Voltage>(VON_Voltage*10))&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
					Von_CONT=1;
				}
				else if((Voltage<(VON_Voltage*10))&&(Von_CONT==0))
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					Von_CONT=0;//��������ż�������
				}
			}
		}
		else if(MODE==5)//LEDģʽ
		{
			static vu32 LED_ON_V;
			LED_ON_V=0;
			LED_ON_V=(LED_VO*100)*LED_RD;
			LED_ON_V=LED_VO-(LED_ON_V/10000);//ͨ��RDϵ�������LED�����ż���ѹ
			if(Voltage>=LED_ON_V)//�жϵ�ѹ�Ƿ����LED��ͨ��ѹ
			{
				Cont_coeff_LEDMODE(LED_ON_V);
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
			}
			else
			{
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
				SET_I_TRAN=0;
				TIME_1MS_OVER=0;//��������־
				TIME_1MS_flag=0;//����ʱ���־
			}
		}
		else if(MODE==6)//��·ģʽ
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
		}else if(MODE == 4){//��̬ģʽ
			if(V_Gear_SW==1)//��ѹ�ߵ�λ
			{
				if(Voltage<VOFF_Voltage)//�жϲ�����ѹ�Ƿ�С��ж�ص�ѹ
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					dynaflagA = 0;
					dynaflagB = 0;
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
				}
				if((Voltage>VON_Voltage)&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
					Von_CONT=1;
				}
				else if((Voltage<VON_Voltage)&&(Von_CONT==0))
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					Von_CONT=0;//��������ż�������
					dynaflagA = 0;
					dynaflagB = 0;
				}
			}
			else if(V_Gear_SW==0)//��ѹ�͵�λVON VOFF��*10����Ϊ�趨����ֻ֧�ָߵ�λ�趨
			{
				if(Voltage<(VOFF_Voltage*10))//�жϲ�����ѹ�Ƿ�С��ж�ص�ѹ
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					dynaflagA = 0;
					dynaflagB = 0;
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
				}
				if((Voltage>(VON_Voltage*10))&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//�򿪸���
					Von_CONT=1;
				}
				else if((Voltage<(VON_Voltage*10))&&(Von_CONT==0))
				{
					onoff_ch = 0;
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//�رո���
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//������ֵ��������ÿ�δ򿪸��غ�����������
					TIME_1MS_OVER=0;//��������־
					TIME_1MS_flag=0;//����ʱ���־
					Von_CONT=0;//��������ż�������
					dynaflagA = 0;
					dynaflagB = 0;
				}
			}
		}
	}
	I_SW_COTNR();//������λ�л�
	V_SW_COTNR();//��ѹ��λ�л�
  Sence_SW_CONT();//SENC�л�
}
/***************************************
������:Cont_coeff_LEDMODE
��������:
��������:LEDģʽ���Թ�ϵϵ������
****************************************/
void Cont_coeff_LEDMODE(vu32 on_v)
{
	vu32 var16;
	vu32 var32;
	vu32 var32b;
	vu32 cont_A_READ,cont_B_READ;
	vu32 cont_A_ACT,cont_B_ACT;
	vu32 REG_LED_CorrectinonA;
	vu32 REG_LED_Offset;
	vu8 Polar_led;
/***********��������ϵ��***************/
	cont_A_READ = on_v;
	cont_A_ACT = 0;

	cont_B_READ = LED_VO;
	cont_B_ACT = LED_IO;
	
	var32 = cont_B_ACT;
	var32 = var32 - cont_A_ACT;
	var32 = var32 << 12;
	var16 = cont_B_READ - cont_A_READ;
	var32 = var32 / var16;
	REG_LED_CorrectinonA = var32;
	var32 = cont_B_ACT;
	var32 = var32 << 12;
	var32b = cont_B_READ;
	var32b = var32b * REG_LED_CorrectinonA;
	if (var32 < var32b)
	{
		var32b = var32b - var32;
		REG_LED_Offset = var32b;
		Polar_led |= 0x01;
	}
	else 
	{
		var32 = var32 - var32b;
		REG_LED_Offset = var32;
		Polar_led &= ~0x01;						
	}
/*******************���Ե�������***************************/
	var32=0;
	var32 = Voltage;
	var32 = var32 * REG_LED_CorrectinonA;  //Y=KX+B �������Y=KX
	if ((Polar_led & 0x01) == 0x01)		  //
	{
		if (var32 < REG_LED_Offset) 
		{
			var32 = 0;
		}
		else var32 = var32 - REG_LED_Offset;
	}
	else var32 = var32 + REG_LED_Offset;
	var32 = var32 >> 12;
	SET_R_Current = var32;
	var32 = 0;
}
/***************************************
������:MAXPAR_limit
��������:
��������:���趨�����������
****************************************/
void MAXPAR_limit(void)
{
	if(SET_Voltage>MAX_V)
	{
		SET_Voltage=MAX_V;
	}
	if(SET_Current>MAX_I)
	{
		SET_Current=MAX_I;
	}
	if(VON_Voltage>MAX_V)
	{
		VON_Voltage=MAX_V;
	}
	if(VOFF_Voltage>MAX_V)
	{
		VOFF_Voltage=MAX_V;
	}
	if(I_Rise_Time>20000)
	{
		I_Rise_Time=20000;
	}
	if(CV_Down_Time>3000)
	{
		CV_Down_Time=3000;
	}
	if(I_Down_Time>20000)
	{
		I_Down_Time=20000;
	}
}