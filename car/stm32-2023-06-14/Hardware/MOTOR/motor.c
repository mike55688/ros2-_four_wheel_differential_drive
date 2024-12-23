#include "motor.h"
void Motor_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB2ENR|=1<<4;       //PORTC时钟使能   
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能 
	RCC->APB2ENR|=1<<2;       //PORTA时钟使能 
	
	GPIOA->CRL&=0XFF00FFFF;   // 
	GPIOA->CRL|=0X00220000;   // PA4 5推挽输出
	
	GPIOC->CRL&=0XFFF0000F;   // 
	GPIOC->CRL|=0X00022220;   // PC1 2 3 4推挽输出

	GPIOB->CRL&=0XFFFFFFF0;   // PB0清除原设置
	GPIOB->CRL|=0X00000002;   // PB0推挽输出
	
	GPIOB->CRH&=0X0FFFFFFF;   // PB15清除原设置
	GPIOB->CRH|=0X20000000;   // PB15推挽输出
		
	
	INA1=0;
	INB1=0;
	INC1=0;
	IND1=0;

	INA2=0;
	INB2=0;
	INC2=0;
	IND2=0;
	
	RCC->APB2ENR|=1<<13;       //使能TIM8时钟
	RCC->APB2ENR|=1<<4;        //PORTC时钟使能
	
	GPIOC->CRL&=0X00FFFFFF;    //PORTC 6 7  复用输出
	GPIOC->CRL|=0XBB000000;    //PORTC 6 7  复用输出
	GPIOC->CRH&=0XFFFFFF00;    //PORTC 8 9 复用输出
	GPIOC->CRH|=0X000000BB;    //PORTC 8 9 复用输出
		
	TIM8->ARR=arr;             //设定计数器自动重装值 
	TIM8->PSC=psc;             //预分频器不分频
			
	TIM8->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM8->CCMR1|=6<<12;        //CH2 PWM1模式	
	TIM8->CCMR2|=6<<4;         //CH3 PWM1模式	
	TIM8->CCMR2|=6<<12;        //CH4 PWM1模式	
	
	TIM8->CCMR1|=1<<3;         //CH1预装载使能	  
	TIM8->CCMR1|=1<<11;        //CH2预装载使能	 
	TIM8->CCMR2|=1<<3;         //CH3预装载使能  
	TIM8->CCMR2|=1<<11;        //CH4预装载使能	  
	
	TIM8->CCER|=1<<0;         //CH1输出使能	
 	TIM8->CCER|=1<<4;         //CH2输出使能	   
	TIM8->CCER|=1<<8;         //CH3输出使能	 
	TIM8->CCER|=1<<12;        //CH4输出使能
	
	TIM8->CCR1=0;
  TIM8->CCR2=0;
	TIM8->CCR3=0;
	TIM8->CCR4=0;
	TIM8->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM8->CR1=0x0080;          //ARPE使能 
	TIM8->CR1|=0x01;          //使能定时器	
	
} 
void Steering_engine_PWM_Init(u16 arr,u16 psc)
{		 					 
	RCC->APB2ENR|=1<<11;       //使能TIM1时钟
	RCC->APB2ENR|=1<<2;        //PORTA时钟使能
	GPIOA->CRH&=0XFFFF0FF0;    //PORTA8 11复用输出
	GPIOA->CRH|=0X0000B00B;    //PORTA8 11复用输出
		
	TIM1->ARR=arr;             //设定计数器自动重装值 
	TIM1->PSC=psc;             //预分频器不分频
			
	TIM1->CCMR1|=6<<4;         //CH1 PWM2模式	
	
	TIM1->CCMR2|=6<<12;        //CH4 PWM2模式	
	
	TIM1->CCMR1|=1<<3;         //CH1预装载使能	  
	  
	TIM1->CCMR2|=1<<11;        //CH4预装载使能	  
	
	TIM1->CCER|=1<<0;         //CH1输出使能	
 
	TIM1->CCER|=1<<12;        //CH4输出使能
	TIM1->CCR1=0;

	TIM1->CCR4=0;
	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM1->CR1=0x0080;          //ARPE使能 
	TIM1->CR1|=0x01;          //使能定时器					
} 
