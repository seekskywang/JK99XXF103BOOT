/******************** (C) COPYRIGHT 2015 AVER********************
 * �ļ���  ��KEY.C
 * ����    ������
 * ����    ��3*2�������ɨ��
 * ����    ��ɨ����ɺ����ֵ
 * Ӳ�����ӣ�һ·3*2�������  һ·��������������
 * �޸����ڣ�2015-08-18
********************************************************************/
#include "my_register.h"
#include "beep.h"
#include "flash.h"
#include "gpio.h"
#include "key.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//=================================================================
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
} flagA, flagB,flagC,flagD,flagE,flagF,flagG;
//====================================================================

#define MAX_VOL  3200		
#define MAX_CUR  5100	
vu8 t_KeyScan;
vu32 NewKey;
vu8 t_beep;
vu8 t_lock;
vu8 t_onoff;
vu16 KeyCounter;
vu16 Time_sw;//��ʱ
vu16 Test_Daley;//�Զ������жϵȴ�ʱ�䣬��ʱ���û�������
vu16 Test_C_Door;//�Զ������ż�����
/* ���尴��ֵ */
#define        KEY_ON_OFF                0X01      //
#define        KEY_SET                   0X02      //
#define        KEY_Menu                  0X03      //
#define        KEY_ESC                   0X04      //

/* ���尴��״ֵ̬ */
#define        KEY_NULL_VALUE             0X00    //�޶���
#define        KEY_PRESS                  0X10    //����
#define        KEY_LONG                   0X20    //����
#define        KEY_CONTINUE               0X40    //����
#define        KEY_UP                     0X80    //����̧��
/* ���尴�������� */
#define        KEY_INIT_STATE               0      //��̬
#define        KEY_WOBBLE_STATE             1      //����
#define        KEY_PRESS_STATE              2      //��������
#define        KEY_LONG_STATE               3      //��������
#define        KEY_CONTINUE_STATE           4      //��������
#define        KEY_RELEASE_STATE            5      //�ָ���̬
/* ����ʱ�� */
#define         KEY_LONG_PERIOD             50                /* ����ʱ��1S */
#define        KEY_CONTINUE_PERIOD          10                /* ˫��ʱ��500ms */
/************************************************************************************************************************/
