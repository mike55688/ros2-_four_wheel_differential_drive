#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMA   TIM8->CCR2  
#define PWMB   TIM8->CCR1 
#define PWMC   TIM8->CCR4 
#define PWMD   TIM8->CCR3

#define INA1   PCout(3) 
#define INB1   PCout(1)
#define INC1   PBout(0)
#define IND1   PAout(4)

#define INA2   PCout(4)   
#define INB2   PCout(2)  
#define INC2   PBout(15)
#define IND2   PAout(5)

 
void Motor_PWM_Init(u16 arr,u16 psc);
void Steering_engine_PWM_Init(u16 arr,u16 psc);
#endif
