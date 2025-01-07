#ifndef __ULTRA_H
#define __ULTRA_H	 
#include "sys.h"		 
void TIM1_Cap_Init(u16 arr,u16 psc);
int Get_distance(void);
void TIM1_CC_IRQHandler(void);
#endif
