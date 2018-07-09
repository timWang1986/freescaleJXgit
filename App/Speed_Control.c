#include "include.h"
#include "Speed_Control.h"
#include <math.h>

 float   Kp_Spd,
		  Ki_Spd,
		  Kd_Spd,
		  speed_low,
		  speed_high;

uint16 speed_line[2];

int16 S_Error,Err_New,Err_Old,Last_Err_Old;
int16
      FTM_count=0,
      EC_num=0,
      Speed_Ek0 = 0, Speed_Ek1 = 0, Speed_Ek2 = 0;

volatile uint16 
    Goal_Speed=0;                                        // �Ӽ��ٴ����õ���Ŀ���ٶ�
    
int16 
    Speed_Out=0;                                      //  ���������������
   
uint16 
    Duty_Go    =0,                                   //���������ռ�ձ� ǰ
    Duty_Back  =0;                                   //���������ռ�ձ� ��
float a,b;    
int16 Fuzzy_Speed(void);    
    
void  Encoder_Read(void)
{
   FTM_count = ftm_quad_get(FTM2);          //��ȡFTM �������� ��������(������ʾ������)
   FTM_count = -FTM_count;
   if(FTM_count<0)
     FTM_count=0;
   ftm_quad_clean(FTM2); 
   
  // FTM_count = ftm_input_get (FTM2, FTM_CH0) ;//���׽ ������ֵ
  // ftm_input_clean (FTM2);

} 

int16 Speed_Set(void)
{
  S_Error=ave_center_s-40;

  if(S_Error<0)
  {
    S_Error=-S_Error;
  }
  if(S_Error>20)
  {
    S_Error=20;
  }                                                   //���ٶ�ƫ���޶���0��20֮��


    a=(speed_high-speed_low)/20;
    b=speed_high;
    Goal_Speed=(uint16)(-a*S_Error+b);                           //�ٶȷ�����   //speed=-a*S_Error+b

  /*
     if((Goal_Speed-(uint16)speed_low)>(uint16)((speed_high-speed_low)*bian_bi))         //�����ٶ�ʱ ��ֹ�������ȼ���  ����ɨ�赽��������ȷ���ٶ� 
     {
       Goal_Speed=(uint16)((speed_high-speed_low)*bian_bi+speed_low);              //��������޷�
     }
  
    
     if((Goal_Speed-(uint16)speed_low)>(uint16)((speed_high-speed_low)*bian_ca))         //��������޷�
     {
       Goal_Speed=(uint16)((speed_high-speed_low)*bian_ca+speed_low); 
     }
*/
  
  return Goal_Speed;
}

void  SpeedControl(void) 
{
   Speed_Ek0 = Goal_Speed - FTM_count;
   Speed_Out= (int16)
              (
              Speed_Out
              + (int16)( Kp_Spd * (Speed_Ek0 - Speed_Ek1) )
              + (int16)( Ki_Spd * (Speed_Ek0) )
              + (int16)( Kd_Spd * (Speed_Ek0 - 2*Speed_Ek1 + Speed_Ek2) )
              );
   Speed_Ek2 = Speed_Ek1;
   Speed_Ek1 = Speed_Ek0;
   
   if       ( Speed_Out > 5000 )                     Speed_Out  = 5000;
   else if ( Speed_Out< (int16) -5000 )            Speed_Out  = -5000;   
    
   if(Speed_Out > 0)
        {
            Duty_Go   = (uint16) Speed_Out;
            Duty_Back = 0;
        }
        else
        {
            Duty_Go   = 0;
            Duty_Back = (uint16) (-Speed_Out);
        } 

 /*if((flag_stopcar==1)||(FTM_count>400))           //ͣ������
 {
   if(FTM_count>20)
   {
     Duty_Go   = 0;
     Duty_Back =1500;
   }
   else
   {
     Duty_Go   = 0;
     Duty_Back = 0;
   }
 }*/
 
ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(uint16)Duty_Go);      // ��� PWM
ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(uint16)Duty_Back);      //��� PWM

ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);      // ��� PWM
ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);      //��� PWM

 speed_line[0]=(uint16)FTM_count;
 speed_line[1]=(uint16)Goal_Speed;
 
 
}    