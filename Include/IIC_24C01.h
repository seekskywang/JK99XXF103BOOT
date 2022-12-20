#ifndef _IIC_24C01_H
#define _IIC_24C01_H

#include "my_register.h"

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(vu8 _ucByte);
vu8 i2c_ReadByte(void);
vu8 i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
vu8 i2c_CheckDevice(vu8 _Address);
void i2c_CfgGpio(void);
#endif
