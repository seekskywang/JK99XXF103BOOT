/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * ����    :modbus.h
 * ����    :����
 * ����    :
 * �������:
 * Ӳ������:
 * �޸�����:2015-2-5
*******************************************************************/
#include "my_register.h"	

#ifndef _modbus_h_
#define _modbus_h_

void UART_Action(void);
void UART1_Action(void);
void UART3_Action(void);
void Transformation_ADC(void);
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
vu16 Hardware_CRC_Re(vu8 *p_buffer,vu8 count);
#endif
