#include "sys.h"
/*********************************************
作者：星洛智能
淘宝店铺：http://shop180997663.taobao.com/
*********************************************/
/*******************
以笛卡尔坐标系为参考
Roll为绕X轴旋转角，
Pitch为绕Y轴旋转角，
Yaw为绕Z轴旋转角。
*******************/
int velocityA_Control(int encoder,int target);
int velocityB_Control(int encoder,int target);
int velocityC_Control(int encoder,int target);
int velocityD_Control(int encoder,int target);

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void Abnormal_state_handling(void);
void  APP_Control(void);//APP指令
void  PS2_Control(void);//手柄指令
void  Key(void);//按键指令
void Motion_control(void);
void Usart_rt_data(void);
int Go_straight(float Angle,float Target);
void  send_data_to_blue(void);
u8  Start_Flag=0;                    //启动标志
u8  Start_Flag_one=1;                //启动标志
u8  Start_Flag_STOP=0;               //启动标志
u8  show_flag=0;                     //显示标志
u8  delay_30,delay_flag;             //30毫秒标志

u8 flag_move=0;//
u8 flag_action=0;//

u8 PS2_KEY,PS2_LX,PS2_LY,PS2_RX,PS2_RY;
 
u16 Flag_Z1 ,Flag_Z2 ,Flag_Z3  ;//标志位

u16 FLAG_USART_ON,Count_stop;

int Voltage;//电池电压
int count_size,count_sum;
int Motor_A,Motor_B,Motor_C,Motor_D;         //电机PWM变量
int Last_Motor_A,Last_Motor_B,Last_Motor_C,Last_Motor_D;         //电机PWM变量

int Velocity_center=300;//设定车子速度，默认速度为150
float Velocity_target_A;
float Velocity_target_B;
float Velocity_target_C;
float Velocity_target_D;

float Encoder_A,Encoder_B,Encoder_C,Encoder_D;   //编码器的脉冲计数 

float Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw ,accel_x,accel_y,accel_z;//XYZ三轴旋转角度、角速度

float Velocity_A;		
float Velocity_B;
float Velocity_C;
float Velocity_D;

float Yaw_target;

float line_KP=15,line_KD=25; //直线行走控制PD参数

float Velocity_B_KP=1.5,Velocity_B_KI=10;         //电机B速度控制PI参数


int main(void)
{ 
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	
	Motor_PWM_Init(7199,0);         //初始化舵机PWM 10KHZ 
	
	OLED_Init();                    //OLED初始化
	LED_Init();                     //LED初始化
	KEY_Init();                     //按键初始化
	usart1_init(72,115200);       //串口1初始化
	usart2_init(36,9600);           //串口2初始化

  delay_ms(100);
	Dma_Init(1,DMA1_Channel4,(u32)&USART1->DR,(u32)send_buf,65);//初始化DMA1：(通道4，数据源send_buf[]，传输目标：外设地址 &USART1->DR	，传输数据量),非循环模式	 
	USART1->CR3|=1<<7;                 //串口1DMA中断使能
 	Dma_Enable(DMA1_Channel4,65);      //DMA搬运使能，(通道4 ,传输数据量)
	delay_ms(20);                   //延时
	
	Encoder_Init_TIM2();            //初始化编码器
	Encoder_Init_TIM3();            //初始化编码器
	Encoder_Init_TIM4();            //初始化编码器
	Encoder_Init_TIM5();            //初始化编码器
	
	Adc_Init();                     //adc初始化	
	delay_ms(100);                  //延时

	IIC_Init();                     //IIC初始化
  delay_ms(10);                   //延时		
  MPU6050_initialize();           //MPU6050初始化	
  delay_ms(10);                   //延时	
  DMP_Init();                     //初始化DMP
  delay_ms(400);                  //延时
	PS2_Init();                     //PS2手柄初始化
	PS2_SetInit();                  //PS2手柄初始化	
	

	delay_ms(100);                  //延时		
  EXTI_Init();                    //MPU6050 5ms定时中断初始化
	while(1)
		{		

			  show_flag++;                  //错开OLED与上位机
			  if(show_flag==1) send_data_to_blue();	
				if(show_flag>=5) oled_show();          //oled显示 
			  if(show_flag==7)show_flag=0;
		
			  delay_flag=1;	                         //30ms中断标志位
				while(delay_flag);                     //进入内循环，等待5毫秒中断将 delay_flag标志位置0后跳出    		
		} 
}

/**************************************************************************
MPU6050 INT引脚5毫秒中断	 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		EXTI->PR=1<<12; //清除LINE5上的中断标志位  
		
		 //****************传感器数据获取****************************//
		 Encoder_A = (float)Read_Encoder(5);//获取编码器数值
		 Encoder_B = (float)Read_Encoder(3);//获取编码器数值	
		 Encoder_C = (float)Read_Encoder(2);//获取编码器数值
		 Encoder_D = (float)Read_Encoder(4);//获取编码器数值			
     Read_DMP();//获取角度信息

		 Usart_rt_data();	//串口数据收发处理
				
		//*******************指令控制*****************//
		 Key();//按键检测		
		 APP_Control();//APP蓝牙指令
		 PS2_Control();//PS2指令
		
		 Motion_control();//运动姿态控制 
 		
		 if(Start_Flag==1)
		   {			 
			  Motor_A = +velocityA_Control(+Encoder_A, +Velocity_target_A);
			  Motor_B = +velocityB_Control(+Encoder_B, -Velocity_target_B);
			  Motor_C = +velocityC_Control(+Encoder_C, -Velocity_target_C);				 
			  Motor_D = +velocityD_Control(+Encoder_D, +Velocity_target_D);
			}
		 if(Start_Flag==0)
			 {
          velocityA_Control(0,0);		
          velocityB_Control(0,0);		
          velocityC_Control(0,0);						 
          velocityD_Control(0,0);			 
		   } 
		 
//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓异常状态关闭电机，如电机异常，电量异常，通讯异常↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓//
			
 			Abnormal_state_handling();
		
//↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑异常状态关闭电机，如电机异常，电量异常，通讯异常↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑//	
				
		
    //****************电机控制****************************// 		 					
		 if(Start_Flag==1&& Start_Flag_one==1&& Start_Flag_STOP==0)
		   {			
			  Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);	//PWM赋值
		   }
		 else //关闭电机，清除部分参数状态
			 {
				Motor_A=0;Motor_B=0;
				Motor_C=0;Motor_D=0;				 
		    Set_Pwm(0,0,0,0);
				Yaw_target= Yaw;					 
			 }
 			       					
//************************************其它**************************************************//	
    if(Voltage<1160&&Voltage>600)Led_Flash(20);//电量不足时LED灯快速闪烁
		else if(Voltage<2290&&Voltage>1800)Led_Flash(20);//电量不足时LED灯快速闪烁	 
	  else Led_Flash(100);//led闪烁 
		count_sum+=Get_battery_voltage();//电压采样累计
	  count_size++;
		if(count_size==200) Voltage=count_sum/count_size,count_sum=0,count_size=0;//求平均电压 
	 
		if(delay_flag==1)
		  {
			 if(++delay_30==2)delay_30=0,delay_flag=0;  //给主函数提供30ms的精准延时
		  }
			
 }
	 return 0;	 
} 

void Usart_rt_data(void) 
{
	//**********************************串口通讯收发********************************************************************************//

         float A_Velocity,B_Velocity,C_Velocity,D_Velocity;		 
			   A_Velocity = (float)((Encoder_A/16384.f)/19.f)*200.f *0.797f;//速度m/s =（当前脉冲数/一圈脉冲数/减速比）*频率*轮子周长m
			   B_Velocity = (float)((Encoder_B/16384.f)/19.f)*200.f *0.797f;//速度m/s =（当前脉冲数/一圈脉冲数/减速比）*频率*轮子周长m
				 C_Velocity = (float)((Encoder_C/16384.f)/19.f)*200.f *0.797f;//速度m/s =（当前脉冲数/一圈脉冲数/减速比）*频率*轮子周长m
				 D_Velocity = (float)((Encoder_D/16384.f)/19.f)*200.f *0.797f;//速度m/s =（当前脉冲数/一圈脉冲数/减速比）*频率*轮子周长m

							          /*<01>*/data_u[0]  = Start_Flag;     //电机启动开关，1启动 0停止
							          /*<02>*/data_u[1]  = gyro_Roll ;     //X轴角速度原始数值
							          /*<03>*/data_u[2]  = gyro_Pitch ;    //Y轴角速度原始数值 
							          /*<04>*/data_u[3]  = gyro_Yaw ;      //Z轴角速度原始数值
							          /*<05>*/data_u[4]  = accel_x ;       //X轴加速度原始数值
							          /*<06>*/data_u[5]  = accel_y ;       //Y轴加速度原始数值
							          /*<07>*/data_u[6]  = accel_z ;       //Z轴加速度原始数值				 
							          /*<08>*/data_u[7]  = Yaw ;           //Z轴角度
							          /*<09>*/data_u[8]  = A_Velocity ;    //A轮速度
							          /*<10>*/data_u[9]  = B_Velocity ;    //B轮速度			 
							          /*<11>*/data_u[10] = C_Velocity ;    //C轮速度
							          /*<12>*/data_u[11] = D_Velocity ;    //D轮速度	 
							          /*<13>*/data_u[12] = Voltage ;       //电压值*100
							          /*<14>*/data_u[13] = 0 ;             //预留
							          /*<15>*/data_u[14] = 0 ;             //预留			 
				     if(DMA1->ISR&(1<<13))//通道4传输完成标志(（通道号-1）*4+1)
			       {				 
			        Dma_Disenable(DMA1_Channel4);//DMA搬运除能，通道2 	

              Send_data_ROS();//发送15个数据

								//发送数据				
			        DMA1->IFCR|=1<<13;//清空通道4传输完成标志
			        Dma_Enable(DMA1_Channel4,65);//DMA搬运使能，通道1                    
			       }						 
	   if(FLAG_USART==1)//如果串口标志位置1，表示已完成一组数据传输
	     { 		u8 sum=0;		 				 						 
						for(u8 j=0;j<52;j++)sum+=USART1_data[j];//计算校验和	
						if(sum==USART1_data[52]){
														
            //解算串口接收到的数据，因为解算过程耗时比较长，放在中断里解算会影响时序						 
						   Start_Flag    =   b2f(USART1_data[ 4], USART1_data[ 5], USART1_data[ 6], USART1_data[ 7]); //获取启动位
						   Velocity_A    =   b2f(USART1_data[ 8], USART1_data[ 9], USART1_data[10], USART1_data[11]);//获取A轮速度
					     Velocity_B    =   b2f(USART1_data[12], USART1_data[13], USART1_data[14], USART1_data[15]);//获取B轮速度
						   Velocity_C    =   b2f(USART1_data[16], USART1_data[17], USART1_data[18], USART1_data[19]);//获取C轮速度
					     Velocity_D    =   b2f(USART1_data[20], USART1_data[21], USART1_data[22], USART1_data[23]);//获取D轮速度							

						}               
            USART1_data[52]	=0XFF;								
            i_u=0; 
						sum=0; 																														
				    FLAG_USART_ON=0;
					  FLAG_USART=0;//串口标志位置0			 
			 }
		 
}

void Abnormal_state_handling(void){//异常状态处理

	    //****************电机异常处理****************************// 		
    if(
			  (Motor_A>=+7100 && Encoder_A<-100)||(Motor_A<=-7100 && Encoder_A>+100)|| 
  		  (Motor_B>=+7100 && Encoder_B<-100)||(Motor_B<=-7100 && Encoder_B>+100)||  
		    (Motor_C>=+7100 && Encoder_C<-100)||(Motor_C<=-7100 && Encoder_C>+100)||  
		    (Motor_D>=+7100 && Encoder_D<-100)||(Motor_D<=-7100 && Encoder_D>+100)
		  )Flag_Z1++;
		else Flag_Z1=0;
		if(Flag_Z1>300)Start_Flag_one=0;//以上状态持续1.5秒判定负反馈控制失效（电机线或编码器线接反），停止电机，需要重启主控才能解除
			
	 	if(
 			  (abs(Motor_A)>=+7100 &&fabs(Encoder_A)<10)||
			  (abs(Motor_B)>=+7100 &&fabs(Encoder_B)<10)||
		    (abs(Motor_C)>=+7100 &&fabs(Encoder_C)<10)||
		    (abs(Motor_D)>=+7100 &&fabs(Encoder_D)<10)
			)Flag_Z2++;
		else Flag_Z2=0;
		if(Flag_Z2>1000)Start_Flag_one=0;//以上状态持续5秒判定为电机堵转（或编码器损坏），停止电机，需要重启主控才能解除	
		
	    //****************电压异常处理****************************//
    int all_motor_abs;
    all_motor_abs =	abs(Motor_A) + abs(Motor_B) + abs(Motor_C) + abs(Motor_D);	
         if(Voltage<2220 && all_motor_abs<8000)Flag_Z3++;
    else if(Voltage<2150 && all_motor_abs<12000 && all_motor_abs>=8000)Flag_Z3++;
    else if(Voltage<2100 && all_motor_abs<18000 && all_motor_abs>=12000)Flag_Z3++;
    else if(Voltage<2000 && all_motor_abs<29000 && all_motor_abs>=20000)Flag_Z3++;
    else if(Voltage<1700 && all_motor_abs<29000 && all_motor_abs>=20000)Flag_Z3+=20;		
 		else Flag_Z3=0; 
	  if(Flag_Z3>4000)Start_Flag_one=0;//以上状态持续20秒判定为电压异常，停止电机，，需要重启主控才能解除
		
    if(Voltage<700)Start_Flag_STOP=1;//以上状态判定为未接通电池只靠USB口供电情况下启动电机，这是危险的，立马停止
    else Start_Flag_STOP=0;	

    //****************通讯异常处理****************************//

  	if(FLAG_USART_ON<600)FLAG_USART_ON++;//在串口数据处理里面清零，如果数字超过某个数值证明没有接收到符合协议的串口数据	
		
//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓PWM限制，防止烧坏电机↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓//	
			
		     if((Motor_A - Last_Motor_A) > +500) Motor_A = Last_Motor_A + 500;//单次PWM改变量限制
		else if((Motor_A - Last_Motor_A) < -500) Motor_A = Last_Motor_A - 500;//单次PWM改变量限制
		   Last_Motor_A = Motor_A;
		     if((Motor_B - Last_Motor_B) > +500) Motor_B = Last_Motor_B + 500;//单次PWM改变量限制
		else if((Motor_B - Last_Motor_B) < -500) Motor_B = Last_Motor_B - 500;//单次PWM改变量限制
		   Last_Motor_B = Motor_B;
		     if((Motor_C - Last_Motor_C) > +500) Motor_C = Last_Motor_C + 500;//单次PWM改变量限制
		else if((Motor_C - Last_Motor_C) < -500) Motor_C = Last_Motor_C - 500;//单次PWM改变量限制
		   Last_Motor_C = Motor_C;	
		     if((Motor_D - Last_Motor_D) > +500) Motor_D = Last_Motor_D + 500;//单次PWM改变量限制
		else if((Motor_D - Last_Motor_D) < -500) Motor_D = Last_Motor_D - 500;//单次PWM改变量限制
		   Last_Motor_D = Motor_D;				 
			 		
		if(Motor_A>+7100)Motor_A=+7100;//PWM限幅
	  if(Motor_A<-7100)Motor_A=-7100;//PWM限幅
		
		if(Motor_B>+7100)Motor_B=+7100;//PWM限幅
	  if(Motor_B<-7100)Motor_B=-7100;//PWM限幅
		
		if(Motor_C>+7100)Motor_C=+7100;//PWM限幅
	  if(Motor_C<-7100)Motor_C=-7100;//PWM限幅	
		
		if(Motor_D>+7100)Motor_D=+7100;//PWM限幅
	  if(Motor_D<-7100)Motor_D=-7100;//PWM限幅
			
		if(RED_KEY==0)Start_Flag=0;//紧急开关，按下，则停止		
}
/**************************************************************************
按键指令
**************************************************************************/
void Key(void)
{	
static u8 flag=0;
flag=click_N_Double(70);	
if(flag==1&&Start_Flag==0)Start_Flag=1,anjian_app=0,flag=0; //单击一次启动标志位反转
if(flag==1&&Start_Flag==1)Start_Flag=0,anjian_app=0,flag=0; //单击一次启动标志位反转	
	
}
/**************************************************************************
PS2手柄指令
**************************************************************************/
void  PS2_Control(void)
{	
	
static int	I=0;
	    if(I++==10)
			{

				PS2_KEY=PS2_DataKey();//采集PS2按键
			  PS2_LX=PS2_AnologData(PSS_LX);//采集PS2左摇杆X轴
			  PS2_LY=PS2_AnologData(PSS_LY);//采集PS2左摇杆Y轴
			  PS2_RX=PS2_AnologData(PSS_RX);//采集PS2右摇杆X轴
			  PS2_RY=PS2_AnologData(PSS_RY);//采集PS2右摇杆Y轴
				I=0;
			}
			     
                if(PS2_KEY==13)Start_Flag=1;//按键三角启动
			     else if(PS2_KEY==15)Start_Flag=0;//按键交叉停止
			     else if(PS2_KEY==11)Velocity_center=300;//左食指按钮1
			     else if(PS2_KEY==9) Velocity_center=400;//左食指按钮2
			     else if(PS2_KEY==12)Velocity_center=800;//右食指按钮1
			     else if(PS2_KEY==10)Velocity_center=1600;//右食指按钮2
	

		      if(PS2_LY<88  && PS2_LX>88 &&PS2_LX<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 1;//前进
		 else if(PS2_LY>168 && PS2_LX>88 &&PS2_LX<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 2;//后退
//		 else if(PS2_LX<88  && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 7;//左横移
//		 else if(PS2_LX>168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 8;//右横移
		 else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX<10  && PS2_RY>88 &&PS2_RY<168)flag_move = 7;//(右摇杆左拨)左自转			
     else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>245 && PS2_RY>88 &&PS2_RY<168)flag_move = 8;//(右摇杆右拨)右自转
			
		 else if(PS2_LX<88  && PS2_LY<88  && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 3;//左斜上
		 else if(PS2_LX>168 && PS2_LY>168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 6;//右斜下	
			
		 else if(PS2_LX>168 && PS2_LY<88 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 4;//右斜上
		 else if(PS2_LX<88 && PS2_LY>168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 5;//左斜下
			
     else if(PS2_LX>88 &&PS2_LX<168 && PS2_LY>88 &&PS2_LY<168 && PS2_RX>88 &&PS2_RX<168 && PS2_RY>88 &&PS2_RY<168)flag_move = 0;//停止
			
}
/**************************************************************************
APP蓝牙指令
**************************************************************************/
void  APP_Control(void)
{
      if(anjian_app==1)Start_Flag=1;
 else if(anjian_app==2)Start_Flag=0;	
       if(anjian_app==5)Velocity_center=300;
  else if(anjian_app==6)Velocity_center=400;	
  else if(anjian_app==7)Velocity_center=800;	
  else if(anjian_app==8)Velocity_center=1600;	
	
	        if(yaogan_app >=250 && yaogan_app <290)flag_move = 1;//前进
	   else if(yaogan_app >=70  && yaogan_app <110)flag_move = 2;//后退
	
	   else if(yaogan_app >=205 && yaogan_app <245)flag_move = 3;//左斜上
	   else if(yaogan_app >=295 && yaogan_app <335)flag_move = 4;//右斜上
	
	   else if(yaogan_app >=115 && yaogan_app <155)flag_move = 5;//左斜下
	   else if(yaogan_app >=25  && yaogan_app <65 )flag_move = 6;//右斜下
	
		 else if(yaogan_app >=160 && yaogan_app <200)flag_move = 7;//左自转
		 else if( (yaogan_app >=0 && yaogan_app <20) || 
			   (yaogan_app >=340 && yaogan_app <=360) )flag_move = 8;//右自转	
		 else flag_move = 0;
}
/****************************************************************************************
运动控制
****************************************************************************************/
void Motion_control(void)
{
static int Velocity_mid=300;
if(Velocity_mid < Velocity_center)Velocity_mid+=2;
if(Velocity_mid > Velocity_center)Velocity_mid-=4;	
if(Start_Flag ==0||flag_move ==0)Velocity_mid=300;	
	
	if(FLAG_USART_ON>300){
     if(flag_move==1)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = + Velocity_mid;
     if(flag_move==2)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = - Velocity_mid;
		
     if(flag_move==3)Velocity_target_A = + Velocity_mid,
			               Velocity_target_B = + 0.7F*Velocity_mid,
		                 Velocity_target_C = + 0.5F*Velocity_mid,
			               Velocity_target_D = + 0.95F*Velocity_mid;
		
     if(flag_move==4)Velocity_target_B = + Velocity_mid,
			               Velocity_target_A = + 0.7F*Velocity_mid,
		                 Velocity_target_D = + 0.5F*Velocity_mid,
			               Velocity_target_C = + 0.95F*Velocity_mid;

     if(flag_move==5)Velocity_target_A = - Velocity_mid,
			               Velocity_target_B = - 0.7F*Velocity_mid,
		                 Velocity_target_C = - 0.5F*Velocity_mid,
			               Velocity_target_D = - 0.95F*Velocity_mid;
		
     if(flag_move==6)Velocity_target_B = - Velocity_mid,
			               Velocity_target_A = - 0.7F*Velocity_mid,
		                 Velocity_target_D = - 0.5F*Velocity_mid,
			               Velocity_target_C = - 0.95F*Velocity_mid;

     if(flag_move==7)Velocity_target_A = + Velocity_center,
			               Velocity_target_B = - Velocity_center,
		                 Velocity_target_C = - Velocity_center,
			               Velocity_target_D = + Velocity_center;
		
     if(flag_move==8)Velocity_target_A = - Velocity_center,
			               Velocity_target_B = + Velocity_center,
		                 Velocity_target_C = + Velocity_center,
			               Velocity_target_D = - Velocity_center;
		
     if(flag_move==0)Velocity_target_A = Velocity_target_B = Velocity_target_C = Velocity_target_D = 0;		
	} 

  
	 //转换来自ROS的控制指令
	if(FLAG_USART_ON<300){

	 Velocity_target_A = (float)(Velocity_A /0.797f/200.f)*19.f*16384.f;//（速度m/s/轮子周长m/频率）*减速比*一圈脉冲数
	 Velocity_target_B = (float)(Velocity_B /0.797f/200.f)*19.f*16384.f;//（速度m/s/轮子周长m/频率）*减速比*一圈脉冲数
	 Velocity_target_C = (float)(Velocity_C /0.797f/200.f)*19.f*16384.f;//（速度m/s/轮子周长m/频率）*减速比*一圈脉冲数
	 Velocity_target_D = (float)(Velocity_D /0.797f/200.f)*19.f*16384.f;//（速度m/s/轮子周长m/频率）*减速比*一圈脉冲数		
	} 
			
}
/****************************************************************************************
增量式PI控制
****************************************************************************************/
int velocityA_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //速度滤波  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//速度控制	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//参数清零
	  return velocity;
}
/****************************************************************************************
增量式PI控制
****************************************************************************************/
int velocityB_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //速度滤波  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//速度控制	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//参数清零
	  return velocity;
}
/****************************************************************************************
增量式PI控制
****************************************************************************************/
int velocityC_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //速度滤波  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//速度控制	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//参数清零
	  return velocity;
}
/****************************************************************************************
增量式PI控制
****************************************************************************************/
int velocityD_Control(int encoder,int target)
{
	  static float velocity,bias,Last_bias; 	
		bias = target - encoder;       //速度滤波  	
		velocity += Velocity_B_KP*(bias-Last_bias) + bias*Velocity_B_KI/100;//速度控制	
    Last_bias = bias;
	  if(Start_Flag==0)Last_bias = 0,velocity = 0;//参数清零
	  return velocity;
}
/**************************************************************************
函数功能：直线行走时角度朝向调整
**************************************************************************/
int Go_straight(float Angle,float Target)
{  
	static float Last,Bias,Differential;
	       float PWM,Least;
  	Least =Angle-Target;//获取偏差
	  if(Least<+100&&Least>-100)     //临界转换方向
		{
          Bias *=0.8;		           //一阶低通滤波 
          Bias += Least*0.2;	     //一阶低通滤波 
	        Differential=Bias-Last;  //获取偏差变化率
	        Last=Bias;               //保存上一次的偏差
		      PWM=Bias*line_KP+Differential*line_KD; //位置控制 
		}
		else  {//临界转换方向
			     Bias *=0.8;		         //一阶低通滤波 
           Bias += Least*0.2;	     //一阶低通滤波 
	         Differential=Bias-Last; //获取偏差变化率
	         Last=Bias;              //保存上一次的偏差
		       PWM=-(Bias*line_KP+Differential*line_KD);//位置控制 
		      }
    if(Start_Flag==0)Last=0,Bias=0,Differential=0;//停止时各参数清零
	  return PWM;
	
}


/**************************************************************************
函数功能：赋值给PWM寄存器
*************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	  if(motor_a<0)     INA1=1,INA2=0,PWMA=-motor_a;//
	  else              INA1=0,INA2=1,PWMA=+motor_a;//
	
	  if(motor_b<0)     INB1=1,INB2=0,PWMB=-motor_b;//
	  else              INB1=0,INB2=1,PWMB=+motor_b;//
	
		if(motor_c<0)     INC1=1,INC2=0,PWMC=-motor_c;//
	  else              INC1=0,INC2=1,PWMC=+motor_c;//
	
	  if(motor_d<0)     IND1=1,IND2=0,PWMD=-motor_d;//
	  else              IND1=0,IND2=1,PWMD=+motor_d;//
}
void  send_data_to_blue(void)
{

          printf("  A:");					
          printf("%.2fV",((float)Voltage)/100);
					
          printf("B:");						
          printf("%.3f",(float)(Encoder_A-Encoder_B-Encoder_C+Encoder_D)/4.F);
					
          printf("C:");							
          printf("%.1fV",(float)Roll);
					
          printf("D:");						
          printf("%.1fV",(float)Pitch);
					
          printf("E:");						
          printf("%.1fV",(float)Yaw);
					
          printf("F:");						
          printf("%d",0);//发送当前参数值到手机APP 然后在usart.c 文件里面串口2中断函数里接收来自上位机的参数修改


          printf("G");						
          printf("%d",(short)Velocity_B_KP);
          printf("H");						
          printf("%d",(short)Velocity_B_KI);
          printf("I");						
          printf("%d",(short)line_KP);
          printf("J");						
          printf("%d",(short)line_KD);
          printf("K");						
          printf("%d",(short)0);				 
		

          printf("; ");	
	
}

