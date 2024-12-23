#include "motor.h"

void Motor_PWM_Init(u16 arr,u16 psc)
{		

	
	RCC->APB1ENR|=1<<1;       //TIM3时钟使能    
	RCC->APB2ENR|=1<<2;       //PORTA时钟使能 
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能 
	GPIOA->CRL&=0X00FFFFFF;   //PORTA6 7复用输出
	GPIOA->CRL|=0XBB000000;   //PORTB6 7复用输出
	GPIOB->CRL&=0XFFFFFF00;   //PORTB0 1复用输出
	GPIOB->CRL|=0X000000BB;   //PORTB0 1复用输出
	TIM3->ARR=arr;//设定计数器自动重装值 
	TIM3->PSC=psc;//预分频器不分频
	TIM3->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM3->CCMR1|=6<<12;        //CH2 PWM1模式	
	TIM3->CCMR2|=6<<4;         //CH3 PWM1模式	
	TIM3->CCMR2|=6<<12;        //CH4 PWM1模式	
	
	TIM3->CCMR1|=1<<3;         //CH1预装载使能	  
	TIM3->CCMR1|=1<<11;        //CH2预装载使能	 
	TIM3->CCMR2|=1<<3;         //CH3预装载使能	  
	TIM3->CCMR2|=1<<11;        //CH4预装载使能	  
	
	TIM3->CCER|=1<<0;         //CH1输出使能	
	TIM3->CCER|=1<<4;         //CH2输出使能	   
	TIM3->CCER|=1<<8;         //CH3输出使能	 
	TIM3->CCER|=1<<12;        //CH4输出使能

	TIM3->CR1=0x0080;          //ARPE使能 
	TIM3->CR1|=0x01;          //使能定时器3 	
} 

