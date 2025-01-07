#include "Ultra.h"
u16 TIM1CH1_CAPTURE_STA,TIM1CH1_CAPTURE_VAL;
/**************************************************************************
函数功能：定时器1通道1输入捕获初始化

**************************************************************************/
void TIM1_Cap_Init(u16 arr,u16 psc)	
{	
//RCC->APB2ENR|=1<<11;       //使能TIM1时钟
//	RCC->APB2ENR|=1<<2;        //PORTA时钟使能
//	GPIOA->CRH&=0XFFFF0FF0;    //PORTA8 11复用输出
//	GPIOA->CRH|=0X0000B00B;    //PORTA8 11复用输出
//		
//	TIM1->ARR=arr;             //设定计数器自动重装值 
//	TIM1->PSC=psc;             //预分频器不分频
//			
//	TIM1->CCMR1|=6<<4;         //CH1 PWM2模式	
//	
//	TIM1->CCMR2|=6<<12;        //CH4 PWM2模式	
//	
//	TIM1->CCMR1|=1<<3;         //CH1预装载使能	  
//	  
//	TIM1->CCMR2|=1<<11;        //CH4预装载使能	  
//	
//	TIM1->CCER|=1<<0;         //CH1输出使能	
// 
//	TIM1->CCER|=1<<12;        //CH4输出使能
//	TIM1->CCR1=4000;

//	TIM1->CCR4=7000;
//	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
//	TIM1->CR1=0x0080;          //ARPE使能 
//	TIM1->CR1|=0x01;          //使能定时器	


	RCC->APB2ENR|=1<<11;     //TIM1时钟使能     
	RCC->APB2ENR|=1<<2;    	 //使能PORTA时钟   	 
	GPIOA->CRH&=0XFFFF0FF0;  //清空原有设置
	GPIOA->CRH|=0X00003008;  //PA.8 输入 PA.11输出

  TIM1->ARR=arr;  		//设定计数器自动重装值   
	TIM1->PSC=psc;  		//预分频器 
	TIM1->CCMR1|=1<<0;	    //选择输入端 
 	TIM1->CCMR1|=0<<4; 	    // 配置输入滤波器 不滤波
 	TIM1->CCMR1|=0<<2;   	//配置输入分频,不分频 

	TIM1->CCER|=0<<1; 	//上升沿捕获
	TIM1->CCER|=1<<0; 	//允许捕获计数器的值到捕获寄存器中

	TIM1->DIER|=1<<1;   //允许捕获中断				
	TIM1->DIER|=1<<0;   //允许更新中断	
	TIM1->CR1|=0x01;    //使能定时器1

	MY_NVIC_Init(2,1,TIM1_CC_IRQn,2);//设置中断优先级,抢占2，子优先级2，组2
}

/**************************************************************************
函数功能:发射声波，获取回波
**************************************************************************/
int distance;
int Get_distance(void)
{   
 
		   PAout(11)=1;     //发射控制端给高电平      
	     delay_us(15);    //高电平持续时间
	     PAout(11)=0;     //发射控制端电平置低
		if(TIM1CH1_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			distance=TIM1CH1_CAPTURE_STA&0X3F;
			distance*=65536;					    //溢出时间总和
			distance+=TIM1CH1_CAPTURE_VAL;		//得到总的高电平时间
			distance=distance*170/1000;           //距离=接收到高电平持续时间*音速/2（1000是音速的时间单位换算成毫秒）				                       
			TIM1CH1_CAPTURE_STA=0;			    //开启下一次捕获
		}	
		return distance;
}
/**************************************************************************
函数功能：超声波回波脉宽读取中断
**************************************************************************/

void TIM1_CC_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM1->SR;
	if((TIM1CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
				                    {
                                 	 if(tsr&0X01)//溢出
					                            {	    
						                         if(TIM1CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
							                       {
								                    if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
									                  {
										               TIM1CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
										               TIM1CH1_CAPTURE_VAL=65535;
									                  }
								                    else TIM1CH1_CAPTURE_STA++;
							                       }	 
					                             }
				                     if(tsr&0x02)//捕获1发生捕获事件
				    	                        {	
						                         if(TIM1CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
							                       {			
								                    TIM1CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
								                    TIM1CH1_CAPTURE_VAL=TIM1->CCR1;	//获取当前的捕获值.
								                    TIM1->CCER&=~(1<<1);		//CC1P=0 设置为上升沿捕获
							                       }
						                         else  //还未开始,第一次捕获上升沿
				   	                               {         
								                    TIM1CH1_CAPTURE_STA=0;		//清空
								                    TIM1CH1_CAPTURE_VAL=0;     //清空
								                    TIM1CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
								                    TIM1->CNT=0;			//计数器清空
								                    TIM1->CCER|=1<<1; 			//CC1P=1 设置为下降沿捕获
							                       }		    
							                     }			     	    					   
		                              }
	TIM1->SR=0;//清除中断标志位 	     
}
