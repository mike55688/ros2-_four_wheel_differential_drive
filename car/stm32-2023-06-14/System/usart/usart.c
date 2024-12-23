#include "usart.h"	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      

	while((USART2->SR&0X40)==0);
	USART2->DR = (u8) ch;      
  return ch;
}
#endif 

//******************************串口1******************************************//
/////////////////////////////////////////////////////////////////////////////////
//****************usart1发送一个字节************************************//
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
//****************串口1初始化************************************//
void usart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
	GPIOA->ODR|=1<<9;	  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART1_IRQn,2);//抢占优先级2,响应优先级2,组2
}

//******************************串口1接收中断*************************************//


 u8 USART1_data[55];
 u8 i_u;
 u8 FLAG_USART=0;	
int count_ros;
int break_count=0;
int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//接收到数据
	{	      	
   					USART1_data[i_u] = USART1->DR;//读取数据

			switch(i_u)
			{
				case 0:
					if( USART1_data[0]==0XAA)//读到帧头数据
					{
						i_u++;//下一个字节数据
					}
					break;
				case 1:
					if( USART1_data[1]==0XAA)//读到帧头数据
					{
						i_u++;//下一个字节数据
		
					}

					break;				
				case 2:
					if( USART1_data[2]==0XF1)//读到帧头数据
					{
						i_u++;//下一个字节数据
		
					}
					break;	
				case 3:case 4:case 5:case 6:case 7:case 8:case 9:case 10:case 11:case 12: 
				case 13:case 14:case 15:case 16:case 17:case 18:case 19:case 20:case 21:case 22:  
				case 23:case 24:case 25:case 26:case 27:case 28:case 29:case 30:case 31:case 32:  
				case 33:case 34:case 35:case 36:case 37:case 38:case 39:case 40:case 41:case 42: 
				case 43:case 44:case 45:case 46:case 47:case 48:case 49:case 50:   					
				case 51:
          i_u++;
					break;
				case 52://和校验	
          FLAG_USART=1;	
            break_count=0;					
					break;				
				default:
					break;				
			}	
		
   }
return 0;	
}
//************************发送15个数据**************************//
float data_u[15];
#define Len 15
 
void Send_data_ROS(void)
{

    u8 tbuf[Len*4];
    unsigned char *p;
				
    for(u8 i=0;i<Len;i++){
	            p=(unsigned char *)&data_u[i];
        tbuf[4*i+0]=(unsigned char)(*(p+3));
        tbuf[4*i+1]=(unsigned char)(*(p+2));
        tbuf[4*i+2]=(unsigned char)(*(p+1));
        tbuf[4*i+3]=(unsigned char)(*(p+0));
    }		
		
    usart1_sent(0XF1,tbuf,Len*4);//自定义帧,功能码0XF1
}
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多48字节!!
//len:data区有效数据个数

u8 send_buf[66];
void usart1_sent(u8 fun,u8*data,u8 len)
{
    
    
    if(len>60)return;    //最多60字节数据
    send_buf[len+4]=0;  //校验数置零
    send_buf[0]=0XAA;   //帧头
	  send_buf[1]=0XAA;   //帧头
    send_buf[2]=fun;    //功能字
    send_buf[3]=len;    //数据长度
//    send_buf[len+5]= 0X0D;    //帧尾	
    for(u8 i=0;i<len;i++)send_buf[4+i]=data[i];         //复制数据
    for(u8 i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];//计算校验和

}
//4个byte数组转换为一个float数值
float b2f(byte m0, byte m1, byte m2, byte m3)
{
//求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//求阶码
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//求尾码 
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

//******************************串口2********************************//
///////////////////////////////////////////////////////////////////////
//****************************usart2发送一个字节*********************//
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
//******************************串口2初始化**************************//
void usart2_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF; 
	GPIOA->CRL|=0X00008B00;//IO状态设置
  
	RCC->APB1RSTR|=1<<17;   //复位串口1
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART2->CR1|=1<<8;    //PE中断使能
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART2_IRQn,2);//组2，最低优先级 
}

//***************************蓝摇APP通讯*******************//
/*********************************************
作者：星洛智能
淘宝店铺：http://shop180997663.taobao.com/
*********************************************/
void LANYAO_APP(int data)//发送数据到蓝摇APP
{
  static int SEND_DATA[10];	
  int i,j,k,data_c;
if(data==0)usart2_send(0x30);
if(data>0)
	{	
	 i=data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }	
			for(k=0;k<j;k++){SEND_DATA[k]=data%10;data=data/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	if(data<0)
	{
   data_c=-data; 		
	 i=-data;
   while(i)
          {
	        	i=i/10;		
		        j++;
	        }
   usart2_send(0x2D);				
			for(k=0;k<j;k++){SEND_DATA[k]=data_c%10;data_c=data_c/10;}
      for(k=j-1;k>=0;k--){usart2_send(SEND_DATA[k]+0x30);}			
   }
	usart2_send(0x0A),
	usart2_send(0x0D);
  memset(SEND_DATA, 0, sizeof(int)*10);	
} 
//**************************串口2接收中断***********************//
int Usart2_Receive;
int data_app[50];
u8 flag_mode_app=0;
int anjian_app=0,huakuai_app=0,yaogan_app=520;
u8 len_1;
u8 len_2;
u8 len_3;
u8 len_4;
u8 len_5;
u8 len_6;

u32 data;
/*********************************************
作者：星洛智能
淘宝店铺：http://shop180997663.taobao.com/
*********************************************/
int USART2_IRQHandler(void)
{	
 static u8 i=0;
	if(USART2->SR&(1<<5))//接收到数据
	{	   
		      //Usart2_Receive=USART2->DR;		
					data_app[i] = USART2->DR;//读取数据		
			
			            if(data_app[0]==0X79 &&Usart2_Receive==0)Usart2_Receive=1;
					        if(data_app[0]==0XAA &&Usart2_Receive==0)Usart2_Receive=2;
		
		  if(Usart2_Receive==1){//适应旧版的蓝遥APP	
						 switch(i)
			{
				case 0:
						i++;//下一个字节数据
					break;
				case 1:
					if( data_app[1]==0X62)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=0;//按键数据
					}
			   	else	if( data_app[1]==0X76)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=1;//滑块数据
					}
					else	if( data_app[1]==0X64)//读到帧头数据
					{
						i++;//下一个字节数据
						flag_mode_app=2;//摇杆数据
					}

					break;				
				case 2:
						i++;//下一个字节数据
					break;	
				case 3:            
            i++;//下一个字节数据	
					break;
				case 4:
					 if(flag_mode_app==0&&data_app[3]==0X0A&&data_app[4]==0X0D)
					   {
							 Usart2_Receive=0;
							 anjian_app=data_app[2],i=0;//一帧数据完毕
 			         memset(data_app, 0, sizeof(uint8_t)*50);								 
						 }
					 if(flag_mode_app==1&&data_app[3]==0X0A&&data_app[4]==0X0D)
					   {
							 Usart2_Receive=0;
						   huakuai_app=data_app[2],i=0;//一帧数据完毕
 			         memset(data_app, 0, sizeof(uint8_t)*50);							 
						 }
					 if(flag_mode_app==2)i++;
					break;
				case 5:
					 if(data_app[4]==0X0A&&data_app[5]==0X0D)
					   { 							 
							 yaogan_app= data_app[2]+data_app[3];//一帧数据完毕	

					   }
						 Usart2_Receive=0,i=0;
						 memset(data_app, 0, sizeof(uint8_t)*50);						 
					break;
				default:
					break;				
			}
			}
					
		 else if(Usart2_Receive==2){//BLUE APP

				   if(i==7){
          						 
						   len_1=data_app[2]; 
						   len_2=data_app[3];
						   len_3=data_app[4];
						   len_4=data_app[5];
					     len_5=data_app[6];
						   len_6=len_1 + len_2 + len_3 + len_4+ len_5;
					 }
				 if(i>7){
                if(i==len_6+8){ 
                       if(data_app[1]==0XAF && data_app[len_6+7]==0X0A && data_app[len_6+8]==0X0D ){

                          if(len_1>0){
                             data = 0;	
                             for(u8 j=1;j<=len_1;j++) {data += (data_app[j+6]-48) *pow(10,(len_1-j));} 
                             Velocity_B_KP = data;

													}	

                          if(len_2>0){														 
                             data = 0;	
                             for(u8 j=1;j<=len_2;j++) {data += (data_app[len_1+j+6]-48) *pow(10,(len_2-j));} 
														 Velocity_B_KI = data;	
											 
													}	

                          if(len_3>0){														 
                             data = 0;	
                             for(u8 j=1;j<=len_3;j++) {data += (data_app[len_2+len_1+j+6]-48) *pow(10,(len_3-j));} 
														 line_KP = data;

													}

                          if(len_4>0){														 
                             data = 0;	
                             for(u8 j=1;j<=len_4;j++) {data += (data_app[len_3+len_2+len_1+j+6]-48) *pow(10,(len_4-j));} 
														 line_KD = data;

													}
	
                          if(len_5>0){														 
														 data = 0;	 
                             for(u8 j=1;j<=len_5;j++) {data += (data_app[len_4+len_3+len_2+len_1+j+6]-48) *pow(10,(len_5-j));} 
														 //Velocity_B_KP = data;

													}																 
                          
					                len_1=0;
					                len_2=0;
					                len_3=0;
					                len_4=0;
					                len_5=0;
					                len_6=0;														 
				                  Usart2_Receive=0;
													memset(data_app, 0, sizeof(uint8_t)*50);
				                }				 
					            }
				        }
		    i++;
        if(Usart2_Receive==0)	i=0;
			 }
			  
   }
return 0;	

}
//*******************串口3初始化,PCLK2 时钟频率(Mhz),bound:波特率************************//
void usart3_init(u32 pclk2,u32 bound)
{  	 
 float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	

	RCC->APB2ENR|=1<<0;    //开启辅助时钟
	RCC->APB2ENR|=1<<4;   //使能PORTC口时钟  
	RCC->APB1ENR|=1<<18;  //使能串口时钟 
	GPIOC->CRH&=0XFFFF00FF; //IO状态设置
	GPIOC->CRH|=0X00008B00;//PC10输出 PC11输入
	GPIOC->ODR|=1<<10;	 
  AFIO->MAPR|=1<<4;      //部分重映射

	RCC->APB1RSTR|=1<<18;   //复位串口1
	RCC->APB1RSTR&=~(1<<18);//停止复位	   	   
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置	 
	USART3->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART3->CR1|=1<<8;    //PE中断使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//组2，最低优先级 
}


//****************************函数功能：串口3接收中断***************************************//

int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//接收到数据
	{	      	
   	u8 data = USART3->DR;//读取数据
	
   }
return 0;	
}
