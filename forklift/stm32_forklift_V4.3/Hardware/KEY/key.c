#include "key.h"
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	GPIOA->CRH&=0XFFFFFFF0; 
	GPIOA->CRH|=0X00000008;//PA8上拉输入
  GPIOA->ODR|=1<<8; // 上拉	

	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	GPIOA->CRL&=0XF000FFFF; 
	GPIOA->CRL|=0X08880000;//
  GPIOA->ODR|=1<<4; // 上拉
  GPIOA->ODR|=1<<5; // 上拉	
  GPIOA->ODR|=1<<6; // 上拉		

	RCC->APB2ENR|=1<<3;    //使能PORTB时钟	   	 
	GPIOB->CRH&=0XF000FFFF; 
	GPIOB->CRH|=0X08880000;//
  GPIOB->ODR|=1<<12;
  GPIOB->ODR|=1<<13;
  GPIOB->ODR|=1<<14;	
} 
/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 3：长按400个周期后松开
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key,Long_Press=0,count_H=0;	
		static	u16 count_single,Forever_count;
	  if(KEY==0)  count_H=0,Forever_count++;   //长按标志位未置1
    else {if(count_H<10)count_H++;}
	  if(count_H>1)Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//双击执行的指令
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//单击执行的指令
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
			if(Forever_count>300)
			{
			 Long_Press=1;	
			}	
      if(Long_Press==1&&KEY==1)	{Long_Press=0;return 3;}				
		return 0;
}


/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
			static u8 flag_key=1;//按键按松开标志
			if(flag_key&&KEY==0)
			{
			flag_key=0;
			return 1;	// 按键按下
			}
			else if(1==KEY)			flag_key=1;
			return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
			static u16 Long_Press_count,Long_Press;
	    if(Long_Press==0&&KEY==0)  Long_Press_count++;   //长按标志位未置1
      else                       Long_Press_count=0; 
		  if(Long_Press_count>200)		
			{
				Long_Press=1;	
				Long_Press_count=0;
				return 1;
			}				
			 if(Long_Press==1)     //长按标志位置1
			{
				  Long_Press=0;
			}
			return 0;
}
