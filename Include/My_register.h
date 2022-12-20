/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * �ļ���  ��my_register
 * ����    ������
 * ����    ���⺯����
 * ����    ��Define register����ϵͳʹ�õļĴ�����ȫ�ֱ���
 * ע��		 ��ͷ�ļ���ֻ���������������ж���
 * �޸����ڣ�2014-12-9
*********************************************************************/

#include "STM32F10x.h"
#include "me_scpi.h"

#ifndef __my_register_h__
#define __my_register_h__
//=============================================================================

//=============================================================================
#define NOP   __nop();	  //��ָ��

//============================================================================
extern struct bitDefine flagA,flagB,flagC,flagD,flagE,flagF,flagG;
#define   Flag_ADC_Full             (flagA.bit0)//DMA������ɱ�־λ 
#define   Flag_BEEP						    	(flagA.bit1)//������������־
#define   Flag_Key_Scan							(flagA.bit2)//������ʱɨ���־
#define   UART_SEND_flag						(flagA.bit3)//���ڷ�����ɱ�־
#define   UART_Buffer_Rece_flag			(flagA.bit4)//���ڽ�����ɱ�־
#define   Flag_Swtich_ON      			(flagA.bit5)//������������ON��־
#define   Flag_DAC_OFF              (flagA.bit6)//�Ƿ����DAC��־
#define   Flag_Beep_OFF             (flagA.bit7)//�������رձ�־

#define   Flag_I_LOW                (flagB.bit0)//������λ�л���־
#define   flag_CC_MODE							(flagB.bit1)//����ģʽ��־
#define   flag_V_level							(flagB.bit2)//��ѹ��λ�л���־
#define   flag_NOR_CODE							(flagB.bit3)//SCPI����һ֡��ȷ����
#define   flag_Tim_USART            (flagB.bit4)//���ڶ�ʱ����
#define   TIME_1MS_flag						  (flagB.bit5)//1msʱ���־λ
#define   TIME_1MS_OVER				  	  (flagB.bit6)//�������������½���ʱ��־
#define   Flag_AD7655_DMA				    (flagB.bit7)//AD7655_DMA������ɱ�־

#define   Flag_Save_SW				      (flagC.bit0)//������ֵ����EEPROM��־λ

//=============================================================================

#define IAP_FILE_SIZE  		256	//55*1024  	//�����������ֽ��� 200
//--------------------------
extern vu8 DAC_Flag;
extern vu8 Beep_Flag;
extern vu8 t_beep;
extern vu8 t_KEYON;
extern vu16 t_0_5S;
extern vu16 t_1S;
extern vu16 t_5S;
extern vu8 t_USART;
extern vu8 Setcontr_ADDR;//���ò�������FLASHλ��
extern vu8 UART1_Buffer_Rece_flag;
extern vu8 UART3_Buffer_Rece_flag;
//--------------------------
extern vu8 UART_Buffer_Size;//���ڽ������ݳ���
extern vu8 Transmit_BUFFERsize;
//---------------------------
extern vu16 NTC_value;   //ADֵ
extern vu16 NTC1_value;   //ADֵ
extern vu16 Imon_value;
extern vu16 Vmon_value;
extern vu16 Contr_DACVlue;//DAC����ֵ
extern u8 bootflag;
extern u16 USART_RX_CNT;
extern vu8 recupfileflag;
extern u8 uprecbuf[1026];

extern vu32 Transition_Date[10];//���ɱ���
#define SET_V_TRAN                  Transition_Date[0]   //���õ�ѹ���ɱ���
#define SET_I_TRAN                  Transition_Date[1]   //���õ������ɱ���
#define SET_R_Current               Transition_Date[2]   //CRģʽ�������õ���ֵ
#define SET_P_Current				Transition_Date[3]   //CPģʽ�������õ���ֵ
#define SET_S_Current				Transition_Date[4]   //��·ģʽ���ص����õ���ֵ
#define SET_R_LED					Transition_Date[5]   //LEDģʽ�������ü��ص���


extern vu32 Run_Control[55];	
#define Voltage                     Run_Control[0]   //��ѹ����ֵ
#define Current                     Run_Control[1]   //��������ֵ
#define R_DATE                      Run_Control[2]   //�������ֵ
#define Power_DATE                  Run_Control[3]   //���ʲ���ֵ

#define	MES_VOLT_MAX				Run_Control[4]   //������ѹ��ֵVP+
#define	MES_VOLT_MIN				Run_Control[5]  //������ѹ��ֵVP-
#define	MES_VOLT_PTP				Run_Control[6]  //������ѹ���ֵ

#define	MES_CURR_MAX				Run_Control[7]   //����������ֵVP+
#define	MES_CURR_MIN				Run_Control[8]   //����������ֵVP-
#define	MES_CURR_PTP				Run_Control[9]   //�����������ֵ   
/*****************���ϼĴ���Ϊֻ���Ĵ����Ͻ��������**********************/

#define Operation_MODE 				Run_Control[10]   //����ģʽ���� 0-������  1-Զ�̲���(ͨѶ)
#define protect_Flage 				Run_Control[11]   //���ֱ�����־ 1.�����ʱ���  2.���뷴�ӱ�����־  3.�����ѹ������־  4.���ع���������־ 

#define onoff_ch                    Run_Control[12]   //ON/OFF 0-�ر�����  1-�򿪸���  
#define MODE                        Run_Control[13]   //ģʽ����  0-CC  1-CV  2-CR 3-CP 4-��̬ģʽ 5-LED 6-��·ģʽ
#define I_Gear_SW					Run_Control[14]   //������λ�л�����  0-�͵�λ  1-�ߵ�λ
#define V_Gear_SW					Run_Control[15]   //��ѹ��λ�л�����  0-�͵�λ  1-�ߵ�λ
#define ADDR						Run_Control[16]   //������ַ
#define Baud_rate					Run_Control[17]   //������
#define Sence_SW 					Run_Control[18]   //Զ�˲�������

#define SET_Voltage                 Run_Control[19]   //���õ�ѹֵ
#define SET_Current                 Run_Control[20]   //���õ���ֵ
#define SET_Resist                  Run_Control[21]   //���õ���ֵ
#define SET_Power                   Run_Control[22]   //���ù���ֵ
#define OVP_Voltage 				Run_Control[23]   //OVP
#define OCP_Current					Run_Control[24]   //OCP
#define OPP_POWER					Run_Control[25]   //OPP
#define MAX_P						Run_Control[26]   //������ƹ���
#define MAX_V						Run_Control[27]   //������Ƶ�ѹ
#define MAX_I						Run_Control[28]   //������Ƶ���
#define VON_Voltage					Run_Control[29]   //���ص�ѹ
#define VOFF_Voltage				Run_Control[30]   //ж�ص�ѹ
#define I_Rise_Time 				Run_Control[31]   //����������
#define I_Down_Time 				Run_Control[32]   //�����½���
#define CV_Down_Time 				Run_Control[33]   //CVģʽ��ѹ�½�ʱ��

#define LED_VO 						Run_Control[34]   //LEDģʽ˳������ѹ
#define LED_IO 						Run_Control[35]   //LEDģʽ˳��������
#define LED_RD 						Run_Control[36]   //LEDģʽRDϵ�����˲���������������ص�ѹ��ֵ��ΧΪ0-100%�����磺����VOΪ30V RD-0.2  ��ʱ���ص�ѹӦΪ30*0.2=6��VI=30-6=24V��

#define DYNA_Ia 					Run_Control[37]   //��̬ģʽ���ص���A
#define DYNA_Ib 					Run_Control[38]   //��̬ģʽ���ص���B
#define DYNA_Ta   					Run_Control[39]   //��̬ģʽ���س���ʱ��A
#define DYNA_Tb						Run_Control[40]   //��̬ģʽ���س���ʱ��B
#define DYNA_IRise					Run_Control[41]   //��̬ģʽ����������
#define DYNA_IDown					Run_Control[42]   //��̬ģʽ�����½���
#define DYNA_MODE					Run_Control[43]   //��̬ģʽ����ģʽѡ��

#define COMM_MODE					Run_Control[44]   //ͨѶ��ʽѡ��
#define Class_5						Run_Control[45]   //5V
#define Class_9						Run_Control[46]   //9V
#define Class_12					Run_Control[47]   //12V
#define Class_15					Run_Control[48]   //15V
#define Class_20					Run_Control[49]   //20V
#define FILESIZE					Run_Control[51]   //�ļ���С
#define BOOTMODE					Run_Control[52]   //��������ģʽ��0-ֱ�ӽ���APP��1-����BOOTLOADER

/***********���ϼĴ����ɶ���д*****************************************/
typedef struct {
	float TestVL;//������ѹ�͵�k
	float TestVM;//������ѹ�е�k
	float TestVH;//������ѹ�ߵ�k
	
	float SetVL;//���õ�ѹ�͵�k
	float SetVM;//���õ�ѹ�е�k
	float SetVH;//���õ�ѹ�ߵ�k
	
	  
	float OffsetVL;//������ѹ�͵�b
	float OffsetVM;//������ѹ�е�b
	float OffsetVH;//������ѹ�ߵ�b
	
	float OffsetSetVL;//���õ����͵�b
	float OffsetSetVM;//���õ����е�b
	float OffsetSetVH;//���õ����ߵ�b
	
		
/////////////////////////////////////		
	float TestCL;//���������͵�k
	float TestCM;//���������е�k
	float TestCH;//���������ߵ�k
	
	float SetCL;//���õ�ѹ�͵�k
	float SetCM;//���õ�ѹ�е�k
	float SetCH;//���õ�ѹ�ߵ�k
	
	float OffsetCL;//���������͵�b
	float OffsetCM;//���������е�b
	float OffsetCH;//���������ߵ�b
	
	float OffsetSetCL;//���õ����͵�b
	float OffsetSetCM;//���õ����е�b
	float OffsetSetCH;//���õ����ߵ�b
	
/////////////////////////////////////	

}CAL;

typedef union
{
    vu8 CalFlash[96];
    CAL CalPara;
}CalUni; 

extern CAL CalPara;

extern vu32 Correct_Parametet[26];//У׼ϵ��
#define  REG_CorrectionV_LOW       Correct_Parametet[0]
#define  REG_CorrectionV_HIG       Correct_Parametet[1]
#define  REG_CorrectionV_MID_HIG   Correct_Parametet[2]
#define  REG_CorrectionV_MID_HIG1   Correct_Parametet[20]
#define  REG_CorrectionV_MID_HIG2   Correct_Parametet[21]
#define  REG_CorrectionV_MID_HIG3   Correct_Parametet[22]

#define  REG_CorrectionA_LOW       Correct_Parametet[3]
#define  REG_CorrectionA_HIG       Correct_Parametet[4]
#define  REG_CorrectionA_MID_HIG   Correct_Parametet[5]


#define  SET_CorrectionV_LOW       Correct_Parametet[6]
#define  SET_CorrectionV_HIG       Correct_Parametet[7]
#define  SET_CorrectionV_MID_HIG   Correct_Parametet[8]
#define  SET_CorrectionV_MID_HIG1   Correct_Parametet[23]
#define  SET_CorrectionV_MID_HIG2   Correct_Parametet[24]
#define  SET_CorrectionV_MID_HIG3   Correct_Parametet[25]

#define  SET_CorrectionA_LOW       Correct_Parametet[9]
#define  SET_CorrectionA_HIG       Correct_Parametet[10]
#define  SET_CorrectionA_MID_HIG   Correct_Parametet[11]

#define  REG_CorrectionA_LOW2       Correct_Parametet[12]
#define  REG_CorrectionA_LOW3       Correct_Parametet[13]
#define  REG_CorrectionA_HIG1       Correct_Parametet[14]
#define  REG_CorrectionA_HIG2   	Correct_Parametet[15]

#define  SET_CorrectionA_LOW2       Correct_Parametet[16]
#define  SET_CorrectionA_LOW3       Correct_Parametet[17]
#define  SET_CorrectionA_HIG1       Correct_Parametet[18]
#define  SET_CorrectionA_HIG2   	Correct_Parametet[19]

extern vu8 coefficient[13];//У׼ϵ��
#define  Polar0            			 coefficient[0]
#define  Polar1           			 coefficient[1]
#define  Polar2						 coefficient[2]
#define  Polar3						 coefficient[3]
#define  Polar4            			 coefficient[4]
#define  Polar5            			 coefficient[5]
#define  Polar6						 coefficient[6]
#define  Polar7						 coefficient[7]
#define  Polar8						 coefficient[8]
#define  Polar9						 coefficient[9]
#define  Polar10					 coefficient[10]
#define  Polar11					 coefficient[11]
#define  Polar12					 coefficient[12]

extern vu32 Correct_Strong[26];//У׼ϵ��
#define  REG_ReadV_Offset_LOW       Correct_Strong[0]
#define  REG_ReadV_Offset_HIG       Correct_Strong[1]
#define  REG_ReadV_Offset_MID_HIG   Correct_Strong[2]
#define  REG_ReadV_Offset_MID_HIG1  Correct_Strong[20]
#define  REG_ReadV_Offset_MID_HIG2  Correct_Strong[21]
#define  REG_ReadV_Offset_MID_HIG3  Correct_Strong[22]

#define  REG_ReadA_Offset_LOW       Correct_Strong[3]
#define  REG_ReadA_Offset_HIG       Correct_Strong[4]
#define  REG_ReadA_Offset_MID_HIG   Correct_Strong[5]

#define  SET_ReadV_Offset_LOW       Correct_Strong[6]
#define  SET_ReadV_Offset_HIG       Correct_Strong[7]
#define  SET_ReadV_Offset_MID_HIG   Correct_Strong[8]
#define  SET_ReadV_Offset_MID_HIG1  Correct_Strong[23]
#define  SET_ReadV_Offset_MID_HIG2  Correct_Strong[24]
#define  SET_ReadV_Offset_MID_HIG3  Correct_Strong[25]

#define  SET_ReadA_Offset_LOW       Correct_Strong[9]
#define  SET_ReadA_Offset_HIG       Correct_Strong[10]
#define  SET_ReadA_Offset_MID_HIG   Correct_Strong[11]

#define  REG_ReadA_Offset_LOW2       Correct_Strong[12]
#define  REG_ReadA_Offset_LOW3       Correct_Strong[13]
#define  REG_ReadA_Offset_HIG1       Correct_Strong[14]
#define  REG_ReadA_Offset_HIG2   	Correct_Strong[15]

#define  SET_ReadA_Offset_LOW2       Correct_Strong[16]
#define  SET_ReadA_Offset_LOW3       Correct_Strong[17]
#define  SET_ReadA_Offset_HIG1       Correct_Strong[18]
#define  SET_ReadA_Offset_HIG2   	Correct_Strong[19]
//---------------------------
extern vu8 UART_Buffer_Rece[200];
extern vu8 UART_Buffer_Send[200];
extern vu8 UART_Buffer_Rece1[200];
extern vu8 UART_Buffer_Send1[200];
extern vu8 dynaflagA;
extern vu8 dynaflagB;
extern vu8 dynatrigflag;
extern u8 dynaonflag;
extern vu16 SWDelay;

////============================================================================= 

//#define ILOW1   60000   //�����͵�λ����ֵ1
//#define ILOW2   100000   //�����͵�λ����ֵ2
//#define ILOW3   240000   //�����͵�λ����ֵ3

//#define IHIGH1   240000   //�����͵�λ����ֵ1
//#define IHIGH2   600000   //�����͵�λ����ֵ2
//#define IHIGH3   1000000   //�����͵�λ����ֵ3
//#define IHIGH4   2000000   //�����͵�λ����ֵ4

//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
////#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
////#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
//#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   1200000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   240000   //�����͵�λ������Ƶ���
//#define POWER_MAX   2400000   //�����͵�λ������ƹ���
////#define POWER_MAX   800000   //�����͵�λ������ƹ���
////#define POWER_MAX   600000   //�����͵�λ������ƹ���
////=============================================================================
//============================================================================= 
////1200W
//#define ILOW1   30000   //�����͵�λ����ֵ1
//#define ILOW2   50000   //�����͵�λ����ֵ2
//#define ILOW3   100000   //�����͵�λ����ֵ3

//#define IHIGH1   100000   //�����͵�λ����ֵ1
//#define IHIGH2   200000   //�����͵�λ����ֵ2
//#define IHIGH3   300000   //�����͵�λ����ֵ3
//#define IHIGH4   400000   //�����͵�λ����ֵ4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   60000   //�����͵�λ������Ƶ���
//#define POWER_MAX   1200000   //�����͵�λ������ƹ���

////3200W
//#define ILOW1   60000   //�����͵�λ����ֵ1
//#define ILOW2   100000   //�����͵�λ����ֵ2
//#define ILOW3   240000   //�����͵�λ����ֵ3

//#define IHIGH1   240000   //�����͵�λ����ֵ1
//#define IHIGH2   600000   //�����͵�λ����ֵ2
//#define IHIGH3   1000000   //�����͵�λ����ֵ3
//#define IHIGH4   2000000   //�����͵�λ����ֵ4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   150000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   150000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   240000   //�����͵�λ������Ƶ���
//#define POWER_MAX   3200000   //�����͵�λ������ƹ���

////2400W
//#define ILOW1   60000   //�����͵�λ����ֵ1
//#define ILOW2   100000   //�����͵�λ����ֵ2
//#define ILOW3   240000   //�����͵�λ����ֵ3

//#define IHIGH1   240000   //�����͵�λ����ֵ1
//#define IHIGH2   600000   //�����͵�λ����ֵ2
//#define IHIGH3   1000000   //�����͵�λ����ֵ3
//#define IHIGH4   2000000   //�����͵�λ����ֵ4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   240000   //�����͵�λ������Ƶ���
//#define POWER_MAX   2400000   //�����͵�λ������ƹ���

//400W
#define ILOW1   30000   //�����͵�λ����ֵ1
#define ILOW2   50000   //�����͵�λ����ֵ2
#define ILOW3   100000   //�����͵�λ����ֵ3

#define IHIGH1   100000   //�����͵�λ����ֵ1
#define IHIGH2   200000   //�����͵�λ����ֵ2
#define IHIGH3   300000   //�����͵�λ����ֵ3
#define IHIGH4   400000   //�����͵�λ����ֵ4
#define SWDELAY   1000 
//============================================================================= 
#define Receive_BUFFERSIZE   10
#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
#define I_LOW_MAX   40000   //�����͵�λ������Ƶ���
#define POWER_MAX   400000   //�����͵�λ������ƹ���

////600W
//#define ILOW1   30000   //�����͵�λ����ֵ1
//#define ILOW2   50000   //�����͵�λ����ֵ2
//#define ILOW3   100000   //�����͵�λ����ֵ3

//#define IHIGH1   100000   //�����͵�λ����ֵ1
//#define IHIGH2   200000   //�����͵�λ����ֵ2
//#define IHIGH3   300000   //�����͵�λ����ֵ3
//#define IHIGH4   400000   //�����͵�λ����ֵ4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   60000   //�����͵�λ������Ƶ���
//#define POWER_MAX   600000   //�����͵�λ������ƹ���

////800W
//#define ILOW1   30000   //�����͵�λ����ֵ1
//#define ILOW2   50000   //�����͵�λ����ֵ2
//#define ILOW3   100000   //�����͵�λ����ֵ3

//#define IHIGH1   100000   //�����͵�λ����ֵ1
//#define IHIGH2   200000   //�����͵�λ����ֵ2
//#define IHIGH3   300000   //�����͵�λ����ֵ3
//#define IHIGH4   400000   //�����͵�λ����ֵ4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   120000   //��ѹ�͵���ߵ�ѹ
//#define V_HIG_MAX   120000   //��ѹ�ߵ���ߵ�ѹ
//#define I_LOW_MAX   60000   //�����͵�λ������Ƶ���
//#define POWER_MAX   800000   //�����͵�λ������ƹ���
#endif
/******************* (C) COPYRIGHT 2015 KUNKIN *****END OF FILE*************************/
