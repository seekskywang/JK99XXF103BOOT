#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "my_register.h"

void SysTick_Init(void);
void Delay_us(vu32 nTime);         // ��λ1us
void TimingDelay_Decrement(void);
#endif /* __SYSTICK_H */
