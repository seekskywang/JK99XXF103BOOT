/******************************************************************/
/* ���ƣHCLOCK                                                   */
/* Ч������ϵͳHSI��Ϊϵͳʱ�Ӳ�����ϵͳʱ��Ϊ48M                                                       */
/* ���ݣ�                                                        */
/* ���ߣ�����                                                    */
/* ��ϵ��ʽ��QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_rcc.h"
/*****************************************************************/
vu8 HSEStartUpStatus;
void RCC_Configuration(void)
{
  RCC_DeInit();
 
  RCC_HSEConfig(RCC_HSE_ON); 
 
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
  if(HSEStartUpStatus == SUCCESS)        
  {
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
 
    RCC_PCLK2Config(RCC_HCLK_Div1);   
   
		RCC_PCLK1Config(RCC_HCLK_Div2);   
 
    FLASH_SetLatency(FLASH_Latency_2);    
   
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);      
 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);     
   
    RCC_PLLCmd(ENABLE); 
  
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)      
		 {
		 }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
    while(RCC_GetSYSCLKSource() != 0x08)
		 { 
		 }
	 }
}
