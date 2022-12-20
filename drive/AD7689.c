#include "AD7689.h"
#include "GPIO.h"
#include "my_register.h" 
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

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
#define AD7689_CNV_0			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
#define AD7689_CNV_1			GPIO_SetBits(GPIOB,GPIO_Pin_12);
vu16 AD7689_Buffer[8];
vu16 Ad7689_Fit_Imon[100];
vu16 Ad7689_Fit_Vmon[100];
vu16 END_Fit_Imon[20];
vu16 END_Fit_Vmon[20];
void AD7689_InitializeSPI2(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
   /* Enable SPI1 and GPIO clocks */
  /*!< SPI_FLASH_SPI_CS_GPIO, SPI_FLASH_SPI_MOSI_GPIO, 
       SPI_FLASH_SPI_MISO_GPIO, SPI_FLASH_SPI_DETECT_GPIO 
       and SPI_FLASH_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  /*!< SPI_FLASH_SPI Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
 
  
  /*!< Configure SPI_FLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI_CS_PIN pin: SPI_FLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	AD7689_CNV_1;
	AD7689_Delay();
	AD7689_CNV_0;
	//����ֻ���SPI�ڳ�ʼ��
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���16λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
}

vu16 SPI_AD7689_Read(vu16 AD7689_CFG, vu16 AD7689_INx)
{
	vu16 AD7689_Config=0;
	vu16 AD7689_Data;
	vu8 i;
	AD7689_Config = AD7689_CFG<<13|AD7689_INCC<<10|AD7689_INx<<7|AD7689_BW<<6|AD7689_REF<<3|AD7689_SEQ<<1|AD7689_RB;//д�����ò���
	AD7689_Config <<=2;
	AD7689_CNV_0;
  AD7689_Data=AD7689_TransferByte(AD7689_Config);
	AD7689_Delay(); 
	AD7689_CNV_1;
	AD7689_Delay(); 
	return AD7689_Data;
}

/************************************************************************/
/* Function Name:	TM7707_TransferByte.								*/
/* Purpose:			Send of recive a byte.								*/
/* Resources:		SPI controler.										*/
/* Params:																*/
/*	@uiTxData:			The byte data will be sent.						*/
/* Return:			The byte data recived.						+		*/
/* Limitation:		None.												*/
/************************************************************************/
vu16 AD7689_TransferByte(vu16 uiTxData1)
{
	vu16 uiRxData;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,uiTxData1);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	uiRxData = SPI_I2S_ReceiveData(SPI2);
	return uiRxData;
}
/************************************************************************/
/* Function Name:	AD7689_Delay.									*/
/* Purpose:			Wait for device DRDY signal.						*/
/* Resources:		DRDY GPIO pin.										*/
/* Params:			None.												*/
/* Return:			None.												*/
/* Limitation:		None.												*/
/************************************************************************/
void AD7689_Delay(void)
{
	vu16 count ;
  for(count = 1000; count != 0; count--);
}
void AD7689_Delay1(void)
{
	vu16 count ;
  for(count = 1000; count != 0; count--);
}
void AD7689_Scan_CH(void)
{
	vu8 i,f,d;
	vu32 sum1;
	static vu8 var_chI,var_chV;
	static vu8 I_cont,V_cont;
	for(i=0;i<8;i++)
	{
		AD7689_Buffer[i]=SPI_AD7689_Read(1, i);
		Ad7689_Fit_Imon[I_cont]=AD7689_Buffer[3];
		Ad7689_Fit_Vmon[V_cont]=AD7689_Buffer[2];
		
		//Vmon_value=AD7689_Buffer[2];
		//Imon_value=AD7689_Buffer[3];
	}
	I_cont++;
	V_cont++;
	if(I_cont>49)
	{
		I_cont=0;
		sum1=0;
		for(f=0;f<50;f++)
		{
			sum1 +=Ad7689_Fit_Imon[f];
		}
		
		END_Fit_Imon[var_chI]=sum1/100;
		var_chI++;
	}
	if(var_chI>2)
	{
		var_chI=0;
		sum1=0;
		for(f=0;f<3;f++)
		{
			sum1 +=END_Fit_Imon[f];
		}
		Imon_value=sum1/3;
	}
	if(V_cont>99)
	{
		V_cont=0;
		sum1=0;
		for(d=0;d<100;d++)
		{
			sum1 +=Ad7689_Fit_Vmon[d];
		}
		Vmon_value=sum1/100;
	}
}