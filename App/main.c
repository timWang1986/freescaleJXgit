/**************************
200hzʱ��55������  71������.  6290  3710
300      4413



�ҵ��³���
***************************/

#include "common.h"
#include "include.h"


uint16 NeedRow[60] = 
{
20,21,22,23,24,25,26,27,28,30,
32,34,36,38,40,42,44,46,48,50,
53,56,59,62,65,68,71,74,77,80,
84,88,92,96,100,104,108,112,116,120,
125,130,135,140,145,150,155,160,165,170,
176,182,188,194,200,206,212,218,224,230
};//��Ҫ�õ����У�240����ѡ��60��

uint8 yasuo_i,yasuoj;

void  main(void)
{
    Site_t site     = {0, 0};
	
    Size_t imgsize  = {80, 60};             //��ѹͼ���С
	
    Size_t size;                            //lcd��ʾ����ͼ���С
    size.H = 100;
    size.W = 100;	
    LCD_init();	
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
	
    camera_init(imgbuff);
    ftm_pwm_init(S3010_FTM, S3010_CH,300,4423);      //��ʼ�� ��� PWM
	   
    ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,1500);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
	
    ftm_quad_init(FTM2);//��������
	//ftm_input_init(FTM2, FTM_CH0, FTM_Rising,FTM_PS_2);//���׽
    pit_init_ms(PIT1, 10);   
    set_vector_handler(PIT1_VECTORn,PIT1_IRQHandler);   // �����жϷ��������ж���������
    enable_irq(PIT1_IRQn);                         // ʹ��PIT�ж�
    pit_init_ms(PIT0, 10);   
    set_vector_handler(PIT0_VECTORn,PIT0_IRQHandler);   // �����жϷ��������ж���������
    enable_irq(PIT0_IRQn);                         // ʹ��PIT�ж�
	
    while(1)
    {
      camera_get_img();                                   //����ͷ��ȡͼ��
      for(yasuo_i=0;yasuo_i<60;yasuo_i++)	
      {  	
        for(yasuoj=0;yasuoj<240;yasuoj++) //����ͷ�ɼ�ͼ��240*80    ��ѹΪ��ά����60*80 
        {
          if(yasuoj==(NeedRow[yasuo_i]))		
          {		
            img_extract(img[yasuo_i], imgbuff+(NeedRow[yasuo_i]-1)*10, 10); //�������õ����У����½�ѹ��һ��һά������ 		
            break;
          }	  
        }    	
      }		
      image_processing();	
      dir_control();	
     
      
      LCD_Img_gray_Z     (site,size,(uint8 *)img,imgsize); //��ʾ��ά�����ûҶ�ͼ��
      Site_t site_2 = {0,100};   //x = 0 ,y = 100
      LCD_str(site_2,"Ring_Find_Flag", BLUE,WHITE);	
      Site_t site_3 = {116,100};   //x = 0 ,y = 100
      LCD_num_BC(site_3,Ring_Find_Flag,5, BLUE,WHITE);     
      //***********�������************	 	 
      Site_t site_4 = {90,0};  
      LCD_num_BC(site_4,zhuqi_2,10, BLUE,WHITE);	 	
      Site_t site_5 = {90,10};   
      LCD_num_BC(site_5,zhuqi_3,10, BLUE,WHITE);	 
      Site_t site_6 = {105,20}; 	
      LCD_num_BC(site_6,assistr2,5, RED,WHITE);	
      Site_t site_7 = {105,30};  	
      LCD_num_BC(site_7,assistr3,5, BLUE,WHITE); 
      Site_t site_8 = {105,40};  
      LCD_num_BC(site_8,assistr4,5, BLUE,WHITE);
      Site_t site_9 = {105,50};  
      LCD_num_BC(site_9,assistr5,5, BLUE,WHITE);	
    }
}









