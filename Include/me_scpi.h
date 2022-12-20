
#ifndef  __ME_SCPI_H
#define  __ME_SCPI_H

#include "stm32f10x.h"
#include "me_scpi.h"
#include "my_register.h" 

//ת�������ֵ
#define    U8(Halfword) 	(*((   vu8*)(&Halfword)))
#define   U16(Halfword) 	(*((  vu16*)(&Halfword)))
#define   U32(Halfword) 	(*((  vu32*)(&Halfword)))
#define   FLOAT(Halfword) 	(*((float*)(&Halfword)))

/*
-------
��ͷ�ļ�������SCPIҪ�õ������мĴ�����ַ��ʹ�õ�ַʱ��Ҫ�ظ�ʹ��

Run_Control[20]--
--------
*/
extern vu16 Run_temp_contr[10];
#define   StandEventREG    Run_temp_contr[0] //��׼�¼��Ĵ���
#define   QuestEventREG    Run_temp_contr[1] //��ѯ�¼�
#define   OpertEventREG    Run_temp_contr[2] //�����¼�
#define   BitCropeREG      Run_temp_contr[3] //λ��Ĵ���
#define   ErrorCodeREG     Run_temp_contr[4] //�������

#define   BeeperREG  			 Run_temp_contr[6] //�������Ĵ��� 1��on  0��off

#define   LoadONOffREG  	 Run_Control[12] //����ON/OFF
#define   ShortENREG  	   Run_temp_contr[5] //�����·״̬ ON/OFF


#define   LOAD_StartloadV  U32(Run_Control[29]) //���ؿ�ʼ���ص�ѹֵ ���������صĵ�ѹ
#define   LOAD_StartfreeV  U32(Run_Control[30]) //���ؿ�ʼж�ص�ѹֵ 

#define   SET_HIG_DYNCUR   		  U32(Run_Control[64]) //���ö�̬ģʽ��׼λ���ص���  Run_Control[65]
#define   SET_HIG_DYNCURTIM     U32(Run_Control[66])		 //���ö�̬ģʽ��׼λ���ص�������ʱ�� us Run_Control[67]
#define   SET_LOW_DYNCUR   		  U32(Run_Control[68]) //���ö�̬ģʽ��׼λ���ص��� 
#define   SET_LOW_DYNCURTIM     U32(Run_Control[70]) 		 //���ö�̬ģʽ��׼λ���ص�������ʱ�� 
#define   SET_DYN_UPKate        Run_Control[72] 		 //���ö�̬ģʽ������
#define   SET_DYN_DOWNKate      Run_Control[73] 		 //���ö�̬ģʽ�½���
#define   SET_DYN_WorkMode      Run_Control[74] 		 //���ö�̬ģʽ�µĹ���ģʽ

//--------���ع���ģʽ---------------------------------------------------------------
#define   CURR_MODE				 0 //CCģʽ
#define   VOLT_MODE				 1 //CVģʽ
#define   RES_MODE				 2 //CRģʽ
#define   POW_MODE				 3 //CPģʽ
#define   LED_MODE				 4 //LEDģʽ
#define   Baty_MODE				 5 //���ģʽ
#define   DYN_MODE				 6 //��̬����ģʽ


#define   DYN_PULS				 0 //��̬ģʽ�µĹ���ģʽ
#define   DYN_CONT				 1 //��̬ģʽ�µĹ���ģʽ
#define   DYN_TOGG				 2 //��̬ģʽ�µĹ���ģʽ

#define   HIGH_Level			 1 //�ߵ�
#define   LOW_Level			   0 //�͵�

/*
-------
��������ѹ����
MAX CURRENT
MAX VOLOT
--------
*/

#define		MAX_SET_CURRENT       200000    //20.0000A  ������õ��� (�͵� 2.00000A  �ߵ� 20.0000)
#define		MAX_SET_VOLT          150000    //150.000V  ������õ�ѹ (�͵� 15.0000V  �ߵ� 150.000)
#define		MAX_SET_POW           120000    //120.000W  �����
#define		MAX_SET_RES           50000.0   //50000.0R  ������


#define		MAX_CURRENT    				40000			//������ ( 2A )��ͬ��λ����С���㲻ͬ ( 0- 2.0000, 0-20.000)
#define		MIN_CURRENT    				1000			//��С����  �����жϵ�λ����


#define		MAX_CURRENTUP    			30000			//������������
#define		MIN_CURRENTUP   			6			    //��С����������
#define		MAX_CURRENTDOWN    		30000			//�������½���
#define		MIN_CURRENTDOWN  			6			    //��С�����½���
#define		MAX_CURRENTPOOTE  		210000		//����������ֵ ��21.0000��

#define		MAX_POWERPOOTE  		 	15000			//����ʱ���ֵ ��150.00��

#define		MAX_VOLOTE    				150000		//����ѹ  (0-15.0000�� 0-150.000)
#define		MIN_VOLOTE    				100			  //��С��ѹ

#define		MAX_LoadVOLOTE    		150000		//���ʼ���ص�ѹֵ 150.000
#define		MIN_LoadVOLOTE    		0			    //��С��ʼ���ص�ѹֵ

#define		MAX_FreeLoadVOLOTE    120000		//���ʼж�ص�ѹֵ
#define		MIN_FreeLoadVOLOTE    0			    //��С��ʼж�ص�ѹֵ


#define		DYN_HIGH_MAX          210000    //��̬ģʽ��׼λ���ص��� 21.0000  MAX
#define		DYN_HIGH_MIN          0         //��̬ģʽ��׼λ���ص��� 0        MIN
#define		DYN_LOW_MAX           210000    //��̬ģʽ��׼λ���ص��� 21.0000  MAX
#define		DYN_LOW_MIN           0         //��̬ģʽ��׼λ���ص��� 0        MIN


#define		DYN_HIGH_DELLMAX      50000000    //��̬ģʽ��׼λ���ص���ʱ�� 50.00000S  MAX  >10s���һλ��Ч
#define		DYN_HIGH_DELLMIN      10         //��̬ģʽ��׼λ���ص���ʱ�� 0.000010S  MIN

#define		DYN_LOW_DELLMAX      50000000    //��̬ģʽ��׼λ���ص���ʱ�� 50.00000S  MAX >10s���һλ��Ч
#define		DYN_LOW_DELLMIN      10         //��̬ģʽ��׼λ���ص���ʱ�� 0.000010S  MIN

#define		DYN_SLEW_RISEMAX     30000      //��̬ģʽ���������� MAX  3.0000
#define		DYN_SLEW_RISEMIN     6          //��̬ģʽ���������� MAX  0.0006

#define		DYN_SLEW_FALLMAX     30000      //��̬ģʽ�����½��� MAX  3.0000
#define		DYN_SLEW_FALLMIN     6          //��̬ģʽ�����½��� MAX  0.0006




void Me_SCPI_TASK(void);

#endif


/*----------------------------------COPY RIGHT BY XQ_GUO 2019---------------------------------------------*/

