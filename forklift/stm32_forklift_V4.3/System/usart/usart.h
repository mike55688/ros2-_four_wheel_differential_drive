#ifndef __USART_H
#define __USART_H
#include "sys.h"
void usart1_send(u8 data);
void usart1_send_char(u8 c);
void usart1_init(u32 pclk2,u32 bound);

void usart2_send(u8 data);
void usart2_init(u32 pclk2,u32 bound);
void usart3_init(u32 pclk2,u32 bound);

#endif	   















