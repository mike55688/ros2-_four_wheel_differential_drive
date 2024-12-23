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
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      

	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
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
u8 USART1_data[27];
u8 data_len=0;
u8 FLAG_USART=0;	

int USART1_IRQHandler(void)
{	
	if(USART1->SR&(1<<5))//接收到数据
	{	      
			USART1_data[data_len] = USART1->DR;//读取数据
		  let_go=1;
      if(USART1_data[0]==0XAA){
			  data_len++;
				if(data_len>26)FLAG_USART=1,data_len=0;
			}	
   }
return 0;	
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


const uint16_t polynom = 0xA001;
uint8_t Crc_L;
uint8_t Crc_H;
uint16_t Crc; 
uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
    uint8_t i;
    uint16_t crc = 0xffff;
 
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i<8; i++)
        {
            if (crc & 1) {
                crc >>= 1;
                crc ^= polynom;
            }
            else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return(crc);
}
//**************************串口2接收中断***********************//
int Usart2_Receive;
int Usart3_Receive;
u8 data_U2[8];
u8 flag_mode_app=0;
int anjian_app,huakuai_app,yaogan_app=510;
u8 len_2;
u8 Sum_1;

u32 data;
/*********************************************
作者：星洛智能
淘宝店铺：http://shop180997663.taobao.com/
*********************************************/
int USART2_IRQHandler(void)
{	

	if(USART2->SR&(1<<5))//接收到数据
	{	   
    data_U2[len_2]=USART2->DR;
			switch(len_2)
			{ 				
				case 0:
					if(data_U2[0]==0XBB||data_U2[0]==0X01||data_U2[0]==0X02)len_2++;//帧头
					break;
				case 1:
          if(data_U2[1]==0X01||data_U2[1]==0X02||data_U2[1]==0X03||data_U2[1]==0X04)len_2++;//识别位
				  else {len_2=0;memset(data_U2,0,sizeof(7));}//非本机指令，重置参数
					break;				
				case 2:
				case 3:            
				case 4:            
				case 5:                      
          len_2++;
					break;				
				case 6:
					Sum_1=0;
				  if(data_U2[0]==0XBB){//帧头表示丝杆步进电机
            for(uint8_t i=0;i<6;i++)Sum_1+=	data_U2[i];	//前6个数据和校验位
				    if(data_U2[6] == Sum_1){ 
						  if(data_U2[1]==0x01)r_yw1=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x02)r_yw2=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x03)r_yw3=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));
					    if(data_U2[1]==0x04)r_yw4=(int32_t)((data_U2[2]<<0)|(data_U2[3]<<8)|(data_U2[4]<<16)|(data_U2[5]<<24));						
					  }
					}else{//帧头表示电爪步进电机
						Crc=crc16bitbybit(data_U2, 5);//前5个数据进行CRC校验，获得校验码
						Crc_L=Crc>>0;//校验码低8位
						Crc_H=Crc>>8;//校验码高8位
						if(data_U2[0]==0X01&&data_U2[1]==0X03&&Crc_L==data_U2[5]&&Crc_H==data_U2[6]){S_Claw1=data_U2[4];}
						if(data_U2[0]==0X02&&data_U2[1]==0X03&&Crc_L==data_U2[5]&&Crc_H==data_U2[6]){S_Claw2=data_U2[4];}						
					}
					len_2=0;memset(data_U2,0,sizeof(7));//不管校验是否通过，重置参数	
					break;
				default:
					break;				
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
	RCC->APB2ENR|=1<<3;   //使能PORTB口时钟  
	RCC->APB1ENR|=1<<18;  //使能串口时钟 
	GPIOB->CRH&=0XFFFF00FF; 
	GPIOB->CRH|=0X00008B00;//IO状态设置
	GPIOB->ODR|=1<<10;	 
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

					Usart3_Receive = USART3->DR;//读取数据		
			 
   }
return 0;	
}
