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
    Goal_Speed=0;                                        // 加减速处理后得到的目标速度
    
int16 
    Speed_Out=0;                                      //  反馈给电机的脉冲
   
uint16 
    Duty_Go    =0,                                   //最后给电机的占空比 前
    Duty_Back  =0;                                   //最后给电机的占空比 后
float a,b;    
int16 Fuzzy_Speed(void);    
    
void  Encoder_Read(void)
{
   FTM_count = ftm_quad_get(FTM2);          //获取FTM 正交解码 的脉冲数(负数表示反方向)
   FTM_count = -FTM_count;
   if(FTM_count<0)
     FTM_count=0;
   ftm_quad_clean(FTM2); 
   
  // FTM_count = ftm_input_get (FTM2, FTM_CH0) ;//单项捕捉 的脉冲值
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
  }                                                   //将速度偏差限定在0到20之间


    a=(speed_high-speed_low)/20;
    b=speed_high;
    Goal_Speed=(uint16)(-a*S_Error+b);                           //速度方案二   //speed=-a*S_Error+b

  /*
     if((Goal_Speed-(uint16)speed_low)>(uint16)((speed_high-speed_low)*bian_bi))         //计算速度时 防止弯道大幅度加速  根据扫描到的行数来确定速度 
     {
       Goal_Speed=(uint16)((speed_high-speed_low)*bian_bi+speed_low);              //纵向弯道限幅
     }
  
    
     if((Goal_Speed-(uint16)speed_low)>(uint16)((speed_high-speed_low)*bian_ca))         //横向弯道限幅
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

 /*if((flag_stopcar==1)||(FTM_count>400))           //停车控制
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
 
ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(uint16)Duty_Go);      // 电机 PWM
ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(uint16)Duty_Back);      //电机 PWM

ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,2000);      // 电机 PWM
ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);      //电机 PWM

 speed_line[0]=(uint16)FTM_count;
 speed_line[1]=(uint16)Goal_Speed;
 
 
}    