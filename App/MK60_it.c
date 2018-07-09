#include	"common.h"
#include	"include.h"
#include	"MK60_it.h"

#define	D	0.04		//����ֱ�� 0.04M

uint16 speed_temp = 0, speed = 0;





/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
	
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

void PIT0_IRQHandler(void)				//��ʱ���ж�
{
      
 
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}

void PIT1_IRQHandler(void)				//��������ʱ��
{
    //Encoder_Read();
    PIT_Flag_Clear(PIT1);       //���жϱ�־λ
}

/*!
 *  @brief      FTM�����жϷ�����
 *  @since      v5.0
 *  @warning    �˺�����Ҫ�û������Լ�������ɣ�����������ṩһ��ģ��
 *  Sample usage:       set_vector_handler(FTM0_IRQn , FTM1_Input_test_handler);    //�� FTM1_Input_test_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */

void FTM1_Input_handler(void)			//�ⲿ�ж�
{
    uint8 s = FTM1_STATUS;             //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
    uint8 CHn;
    FTM1_STATUS = 0x00;             //���жϱ�־λ

    CHn = 0;
    if( s & (1 << CHn) )
    {
        FTM_IRQ_DIS(FTM1, CHn);     //��ֹ���벶׽�ж�
		
        speed_temp ++;
		
        FTM_IRQ_EN(FTM1, CHn); //�������벶׽�ж�

    }
}