
#include"include.h"

#define S3010_FTM   FTM1
#define S3010_CH    FTM_CH0
#define S3010_HZ    (300)

#ifndef _VCAN_CAMERA_H_
#define _VCAN_CAMERA_H_


#define CAMERA_OV7725_EAGLE         2       //山外鹰眼
#define CAMERA_OV7725_WOLF          3       //山外狼眼


#define USE_CAMERA      CAMERA_OV7725_EAGLE   //选择使用的 摄像头

typedef struct
{
    uint8 addr;                 /*寄存器地址*/
    uint8 val;                   /*寄存器值*/
} reg_s;

//定义图像采集状态
typedef enum
{
    IMG_NOTINIT = 0,
    IMG_FINISH,             //图像采集完毕
    IMG_FAIL,               //图像采集失败(采集行数少了)
    IMG_GATHER,             //图像采集中
    IMG_START,              //开始采集图像
    IMG_STOP,               //禁止图像采集
} IMG_STATUS_e;



  
#include  "VCAN_SCCB.h"
#include  "VCAN_OV7725_Eagle.h"

extern void img_extract(void *dst, void *src, uint32_t srclen);


// 摄像头 接口统一改成 如下模式

//  camera_init(imgaddr);
//  camera_get_img();
//  camera_cfg(rag,val)


//  camera_vsync()  //场中断
//  camera_href()   //行中断
//  camera_dma()    //DMA中断

// 需要 提供 如下 宏定义
// #define  CAMERA_USE_HREF    1     //是否使用 行中断 (0 为 不使用，1为使用)
// #define  CAMERA_COLOR       1     //摄像头输出颜色 ， 0 为 黑白二值化图像 ，1 为 灰度 图像 ，2 为 RGB565 图像
// #define  CAMERA_POWER       0     //摄像头 电源选择， 0 为 3.3V ,1 为 5V


//直道左右扫线
  
extern uint8  img[60][CAMERA_W];
extern uint8  imgbuff[CAMERA_SIZE]; 
extern uint8  Ring_Find_Flag;

extern uint8  assistr0 ;
extern uint8  assistr1 ;//入环岛
extern uint8  assistr2 ;//两边都没丢线
extern uint8  assistr3 ;//环内巡线
extern uint8  assistr4 ;//出环岛
extern uint8  assistr5 ;
extern uint8  assistr6 ;

extern uint8 zhuqi_test;

extern uint8 zhuqi_2;
extern uint8 zhuqi_3;

extern uint16 ave_center_s;
extern uint16 ave_center;
extern uint16 dir;
extern uint8 shi_flag;
extern uint8 left_lost;
extern uint8 right_lost;

void image_processing(void); 
extern void dir_control(void);



#endif


