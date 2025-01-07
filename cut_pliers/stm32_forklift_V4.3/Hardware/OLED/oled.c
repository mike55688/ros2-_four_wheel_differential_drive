#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
//**********************************************************//
void oled_show(void)
{
 		 if(Roll<0)		OLED_ShowString(00,0,"R:-"),OLED_ShowNumber(22,0,-Roll,3,12);
		else					OLED_ShowString(00,0,"R:+"),OLED_ShowNumber(22,0,+Roll,3,12);	
       
  	
		if(Pitch<0)		OLED_ShowString(44,0,"P:-"),OLED_ShowNumber(66,0,-Pitch,3,12);
		else					OLED_ShowString(44,0,"P:+"),OLED_ShowNumber(66,0,+Pitch,3,12);	
	
	  
		if(Yaw<0)		  OLED_ShowString(88,0,"Y:-"),OLED_ShowNumber(110,0,-Yaw,3,12);
		else					OLED_ShowString(88,0,"Y:+"),OLED_ShowNumber(110,0,+Yaw,3,12);		
    

	
	  if( Motor_A<0)		    OLED_ShowString(00,10,"-"),
		                      OLED_ShowNumber(15,10,-Motor_A,5,12);
		else                 	OLED_ShowString(00,10,"+"),
		                      OLED_ShowNumber(15,10, Motor_A,5,12); 
		
		if( Encoder_A<0)		  OLED_ShowString(80,10,"-"),
		                      OLED_ShowNumber(95,10,-Encoder_A,5,12);
		else                 	OLED_ShowString(80,10,"+"),
		                      OLED_ShowNumber(95,10, Encoder_A,5,12);
		
		
	  if( Motor_B<0)		    OLED_ShowString(0,20,"-"),
		                      OLED_ShowNumber(15,20,-Motor_B,5,12);
		else                 	OLED_ShowString(0,20,"+"),
		                      OLED_ShowNumber(15,20, Motor_B,5,12); 
		
		if( Encoder_B<0)		  OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_B,5,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_B,5,12);
		if(gyro_Roll<0)		     OLED_ShowString(00,30,"R-"),OLED_ShowNumber(24,30,(int)(-gyro_Roll),3,12),OLED_ShowString(44,30,"."),OLED_ShowNumber(50,30,((int)(-gyro_Roll*100))%100,2,12);
		else					         OLED_ShowString(00,30,"R+"),OLED_ShowNumber(24,30,(int)(+gyro_Roll),3,12),OLED_ShowString(44,30,"."),OLED_ShowNumber(50,30,((int)(+gyro_Roll*100))%100,2,12);	
       
  	
		if(gyro_Pitch<0)		   OLED_ShowString(66,30,"P-"),OLED_ShowNumber(90,30,-gyro_Pitch,3,12),OLED_ShowString(110,30,"."),OLED_ShowNumber(116,30,((int)(-gyro_Pitch*100))%100,2,12);
		else					         OLED_ShowString(66,30,"P+"),OLED_ShowNumber(90,30,+gyro_Pitch,3,12),OLED_ShowString(110,30,"."),OLED_ShowNumber(116,30,((int)(+gyro_Pitch*100))%100,2,12);	

		if(gyro_Yaw<0)		     OLED_ShowString(00,40,"R-"),OLED_ShowNumber(24,40,(int)(-gyro_Yaw),3,12),OLED_ShowString(44,40,"."),OLED_ShowNumber(50,40,((int)(-gyro_Yaw*100))%100,2,12);
		else					         OLED_ShowString(00,40,"R+"),OLED_ShowNumber(24,40,(int)(+gyro_Yaw),3,12),OLED_ShowString(44,40,"."),OLED_ShowNumber(50,40,((int)(+gyro_Yaw*100))%100,2,12);	
       
		
		if(balance_point<0)		 OLED_ShowString(72,40,"-"),OLED_ShowNumber(78,40,(int)(-balance_point),1,12),OLED_ShowString(84,40,"."),OLED_ShowNumber(90,40,((int)(-balance_point*10))%10,1,12);
		else					         OLED_ShowString(72,40,"+"),OLED_ShowNumber(78,40,(int)(+balance_point),1,12),OLED_ShowString(84,40,"."),OLED_ShowNumber(90,40,((int)(+balance_point*10))%10,1,12);	
		
		
                          OLED_ShowNumber(96,40, Start_Flag1,2,12);	
		                      OLED_ShowNumber(110,40, Start_Flag,2,12);	

		                      OLED_ShowString(13,50,".");
		                      OLED_ShowString(35,50,"V");
		                      OLED_ShowNumber(0,50,Voltage/100,2,12);
		                      OLED_ShowNumber(21,50,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(15,50,0,2,12);
		
		 OLED_ShowNumber(50,50,TRACK_4,1,12);
		 OLED_ShowNumber(62,50,TRACK_3,1,12);
		 OLED_ShowNumber(74,50,TRACK_2,1,12);
		 OLED_ShowNumber(86,50,TRACK_1,1,12);		
		 OLED_ShowNumber(105,50,Distance,4,12);//显示超声波距离

		OLED_Refresh_Gram();//刷新	
	}
void oled_show_st(void)
{
static u16 time=0;
time++;
if(time<25){	
 		OLED_ShowChinese(0,00,0,16,1);//开
		OLED_ShowChinese(18,00,1,16,1);//机

		OLED_ShowChinese(36,00,2,16,1);//后
		OLED_ShowChinese(54,00,3,16,1);//请
		OLED_ShowChinese(72,00,4,16,1);//静
	
		OLED_ShowChinese(90,00,5,16,1);//置
		OLED_ShowChinese(108,00,6,16,1);//几
	
 		OLED_ShowChinese(0,20,7,16,1);//秒
		OLED_ShowChinese(18,20,8,16,1);//等

		OLED_ShowChinese(36,20,9,16,1);//待
		OLED_ShowChinese(54,20,10,16,1);//数
		OLED_ShowChinese(72,20,11,16,1);//据
	
		OLED_ShowChinese(90,20,12,16,1);//稳
		OLED_ShowChinese(108,20,13,16,1);//定
		
	  
	}else{
		OLED_ShowChinese(0,00,14,16,1);//单
		OLED_ShowChinese(18,00,15,16,1);//击
		OLED_ShowChinese(36,00,16,16,1);//进
		OLED_ShowChinese(54,00,17,16,1);//入
		OLED_ShowChinese(72,00,18,16,1);//循
		OLED_ShowChinese(90,00,19,16,1);//迹
		OLED_ShowChinese(108,00,20,16,1);//模
		
		OLED_ShowChinese(0,20,21,16,1);//式
		OLED_ShowChinese(18,20,22,16,1);//长
		OLED_ShowChinese(36,20,23,16,1);//按
		OLED_ShowChinese(54,20,24,16,1);//两
		OLED_ShowChinese(72,20,7,16,1);//秒
		OLED_ShowChinese(90,20,2,16,1);//后
		OLED_ShowChinese(108,20,25,16,1);//松
		
		OLED_ShowChinese(0,40,0,16,1);//开
		OLED_ShowChinese(18,40,26,16,1);//为
		OLED_ShowChinese(36,40,27,16,1);//非
		OLED_ShowChinese(54,40,18,16,1);//循
		OLED_ShowChinese(72,40,19,16,1);//迹	
		OLED_ShowChinese(90,40,20,16,1);//模
		OLED_ShowChinese(108,40,21,16,1);//式			
	}
	
		OLED_Refresh_Gram();//刷新	
}

	//********************************************//
u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//向OLED写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

	  	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//更新显示
}
//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示				 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//得到偏移后的值				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //调用1206字体
		else temp=oled_asc2_1608[chr][t];		 //调用1608字体 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
//用16字体
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   
//显示汉字
//x,y:起点坐标
//num:汉字对应的序号
//mode:0,反色显示;1,正常显示
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1,u8 mode)
{
	u8 m,temp;
	u8 x0=x,y0=y;
	u16 i,size3=(size1/8+((size1%8)?1:0))*size1;  //得到字体一个字符对应点阵集所占的字节数
	for(i=0;i<size3;i++)
	{
		if(size1==16)
				{temp=Hzk1[num][i];}//调用16*16字体

		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((x-x0)==size1)
		{x=x0;y0=y0+8;}
		y=y0;
	}
}
//初始化OLED					    
void OLED_Init(void)
{ 	
  //IO初始化 	 
	RCC->APB2ENR|=1<<2;     	
	GPIOA->CRH&=0X0FFFFFFF;	
	GPIOA->CRH|=0X30000000; 
 	RCC->APB2ENR|=1<<4; 
  RCC->APB2ENR|=1<<0;     	//使能AFIO时钟	
	GPIOC->CRH&=0X000FFFFF; 
	GPIOC->CRH|=0X33300000;
	PWR->CR|=1<<8;	//取消备份区写保护 
	RCC->BDCR&=0xFFFFFFFE;	//外部低俗振荡器关闭 PC14，PC15成为普通IO	 	
	BKP->CR&=0xFFFFFFFE; 	//侵入检测TAMPER引脚作为通用IO口使用 
	PWR->CR&=0xFFFFFEFF;	//备份区写保护  	
	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示
	OLED_WR_Byte(0xD5,OLED_CMD); //设置时钟分频因子,震荡频率
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xA8,OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0X3F,OLED_CMD); //默认0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0X00,OLED_CMD); //默认为0

	OLED_WR_Byte(0x40,OLED_CMD); //设置显示开始行 [5:0],行数.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); //bit2，开启/关闭
	OLED_WR_Byte(0x20,OLED_CMD); //设置内存地址模式
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1,OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA,OLED_CMD); //设置COM硬件引脚配置
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]配置
		 
	OLED_WR_Byte(0x81,OLED_CMD); //对比度设置
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //设置VCOMH 电压倍率
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6,OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //开启显示	 
	OLED_Clear();
}  
