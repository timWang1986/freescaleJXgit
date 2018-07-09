#ifndef __MK60_IT_H__
#define __MK60_IT_H__



#define MOTOR1_IO   PTD15			//电机
#define MOTOR2_IO   PTA19
#define MOTOR3_IO   PTA5
#define MOTOR4_IO   PTA24

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH5
#define MOTOR4_PWM  FTM_CH6

#define MOTOR_HZ    10000//(20*1000)	
#define MOTOR_DUTY  10000

//函数声明
void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PIT0_IRQHandler(void);
void FTM1_Input_handler(void);


extern unsigned short int speed_temp, speed;


#endif  //__MK60_IT_H__
