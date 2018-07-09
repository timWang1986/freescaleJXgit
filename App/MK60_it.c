#include	"common.h"
#include	"include.h"
#include	"MK60_it.h"

#define	D	0.04		//后车轮直径 0.04M

uint16 speed_temp = 0, speed = 0;





/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
	
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

void PIT0_IRQHandler(void)				//定时器中断
{
      
 
    PIT_Flag_Clear(PIT0);       //清中断标志位
}

void PIT1_IRQHandler(void)				//编码器定时器
{
    //Encoder_Read();
    PIT_Flag_Clear(PIT1);       //清中断标志位
}

/*!
 *  @brief      FTM测试中断服务函数
 *  @since      v5.0
 *  @warning    此函数需要用户根据自己需求完成，这里仅仅是提供一个模版
 *  Sample usage:       set_vector_handler(FTM0_IRQn , FTM1_Input_test_handler);    //把 FTM1_Input_test_handler 函数添加到中断向量表，不需要我们手动调用
 */

void FTM1_Input_handler(void)			//外部中断
{
    uint8 s = FTM1_STATUS;             //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
    uint8 CHn;
    FTM1_STATUS = 0x00;             //清中断标志位

    CHn = 0;
    if( s & (1 << CHn) )
    {
        FTM_IRQ_DIS(FTM1, CHn);     //禁止输入捕捉中断
		
        speed_temp ++;
		
        FTM_IRQ_EN(FTM1, CHn); //开启输入捕捉中断

    }
}