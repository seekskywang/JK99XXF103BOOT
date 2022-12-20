/******************************************************************/
/* ���ƣHTIM3 PWM                                                */
/* Ч����                                                        */
/* ���ݣ�����һ��200HZ ��ռ�ձȣ�60.9% ��ռ�ձȣ�30.9%��PWM      */
/* ���ߣ�����                                                    */
/* ��ϵ��ʽ��QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "beep.h"
#include "sys_io_cfg.h"

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

vu16 PWM_VALU;
vu16 VP_T,IP_T;
vu16 PR_T1,PR_T2,PR_T3,PR_T4;
/**************************************************************************************/
//void Temp_Comapre(void)	  //�¶������Ʒ���
//{
//  if((NTC_value<800)||(NTC1_value<500)) //���¶ȱ��� �ϸ���
//	{
//		onoff_ch=0;
//		GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
//		protect_Flage=5;//���¶ȱ���
//	}		
//	if((NTC_value<=2250)||(NTC1_value<=2250))//������
//	{
//		IO_FAN_ON;
//	}
//	else if((NTC_value>=2300)&&(NTC1_value>=2300))//�ط���
//	{
//		IO_FAN_OFF;
//	}
//}

void Temp_Comapre(void)	  //�¶������Ʒ���
{
  if((NTC_value<600)/*||(NTC1_value<2000)*/) //���¶ȱ��� �ϸ���
	{
		onoff_ch=0;
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
		protect_Flage=5;//���¶ȱ���
	}		
	if((NTC_value<=1400)/*||(NTC1_value<=2250)*/)//������
	{
		IO_FAN_ON;
	}
	else if((NTC_value>=1550)/*&&(NTC1_value>=500)*/)//�ط���
	{
		IO_FAN_OFF;
	}
}
/********************************************/
void All_protect(void)
{
	 vu32 temp_power;
	
  /*************�����ʱ���***************/
	if(Power_DATE > POWER_MAX)
	{
		PR_T1=0;
		onoff_ch=0;
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
		protect_Flage=1;//�����ʱ���
	}
	if((I_Gear_SW==0)&&(V_Gear_SW==1))
	{
		temp_power=Current/10;
		temp_power=temp_power*Voltage;
		temp_power=temp_power/100000;
		if(temp_power>MAX_P)//�����ʱ���
		{
			PR_T1++;
			if(PR_T1>2000)
			{
				PR_T1=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//�����ʱ���
			}
		}
	}
	else if((I_Gear_SW==1)&&(V_Gear_SW==0))
	{
		temp_power=Current;
		temp_power=temp_power*(Voltage/10);
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//�����ʱ���
		{
			PR_T2++;
			if(PR_T2>2000)
			{
				PR_T2=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//�����ʱ���
			}
		}
	}
	else if((I_Gear_SW==0)&&(V_Gear_SW==0))
	{
		temp_power=Current/10;
		temp_power=temp_power*(Voltage/10);
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//�����ʱ���
		{
			PR_T3++;
			if(PR_T3>2000)
			{
				PR_T3=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_15);//OFF
				protect_Flage=1;//�����ʱ���
			}
		}
	}
	else if((I_Gear_SW==1)&&(V_Gear_SW==1))
	{
		temp_power=Current;
		temp_power=temp_power*Voltage;
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//�����ʱ���
		{
			PR_T4++;
			if(PR_T4>2000)
			{
				PR_T4=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//�����ʱ���
			}
		}
	}
	
 /**************���ӱ���************************/
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==0)//���ӱ���
	{
		onoff_ch=0;
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//�ر�����
		protect_Flage=2;//���ӱ���
		GPIO_SetBits(GPIOB,GPIO_Pin_8);//�򿪷�����
	}
	else 
	{
		protect_Flage=0;//���ӱ���
		GPIO_ResetBits(GPIOB,GPIO_Pin_8);//�رշ�����
	}
	/************��ѹ����************************/
	if((Voltage>MAX_V)&&(V_Gear_SW==1))//��ѹ����
	{
		VP_T++;
		if(VP_T>5)
		{
			VP_T=0;
			onoff_ch=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
			protect_Flage=3;//��ѹ����
		}
	}
	/***************��������****************/
	if((Current>MAX_I)&&(I_Gear_SW==1))//��������
	{
		IP_T++;
		if(IP_T>5)
		{
			IP_T=0;
			onoff_ch=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
			protect_Flage=4;//��������
		}
	}
}