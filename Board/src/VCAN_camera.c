#include "common.h"
#include "include.h"
#include "VCAN_camera.h"

float dir_zhongzhi;
uint16 center;
uint16  last_error,                                                             //上次的偏差
        dir;
int16   dir_out,
        current_error;
float  last_dir;
float
dir_kp1 = 20,
dir_kp2 = 60,
dir_kd = 8;
uint16  dir_kp = 7;

uint8 zhuqi_test = 0;
uint8 zhuqi_2=0;
uint8 zhuqi_3=0;

uint8  img[60][CAMERA_W];
uint8  Ring_Find_Flag = 0;
uint8  LeftRing_Find_Flag = 0;
uint8  RightRing_Find_Flag = 0;
uint8  Ring_Line_Value = 0; //环岛识别标志位

uint8  assist0 = 0;
uint8  assist1 = 0;//
uint8  assist2 = 0;//
uint8  assist3 = 0;
uint8  assist4 = 0;
uint8  assist5 = 0;
uint8  assist6 = 0;
uint8  assist_ADD = 0;/////////左环岛标志位

uint8  assistr0 = 0;
uint8  assistr1 = 0;//入环岛
uint8  assistr2 = 0;//两边都没丢线
uint8  assistr3 = 0;//环内巡线
uint8  assistr4 = 0;//出环岛
uint8  assistr5 = 0;
uint8  assistr6 = 0;
uint8  assist_rADD = 0;/////////右环岛标志位


uint8  car_stop_flag = 0, podao_flag = 0;
uint8  zhangai = 0;
uint8  obstruct;
uint8  bian0_L;
uint8  bian0_R;
uint8  distence1[60];
uint8  bian_distence[60];
uint8  imgbuff[CAMERA_SIZE];                                                    //定义存储接收图像的数组
uint8  img[60][CAMERA_W];                                                 //解码后的图像数组
uint8  H, W;                                                                    //图像数组行、列数
uint8  i, j, k, l;                                                                //应用变量
uint8  bian_line[120];                                                          //边线数组（奇数为左，偶数为右）        //出界停车               //读取的编码器数据
uint16  ave_center = 40;                                                            //中线的加权平均值
uint16  ave_S_num, S_num;
uint16  ave_center_s = 40;
uint8  last_ave_center;                                                         //上一次的中线加权平均值
uint16 sum_center;                                                              //中线的总和             //当前偏差
int16  error_bian_L[2];
int16  error_bian_R[2];
int16  error2_bian_L;
int16  error2_bian_R;
uint8  bian_flag_R;
uint8  bian_flag_L;
uint8  za_flag_r, za_flag_l, za_flag;
uint8  flag_R;
uint8  flag_L;
uint8  num_ave = 0;                                                             //有效中线个数，用于计算中线加权平均值
uint8  line[60];
uint8  shi_flag = 0;
uint8  bian[120];
uint8  L_stop = 0,
       R_stop = 0,
       L_H = 0,
       R_H = 0;
uint8  tiaobian_R3,
       tiaobian_L3,
       tiaobian_R2,
       tiaobian_L2,
       tiaobian_R1,
       tiaobian_L1,
       tiaobian_0,
       tiaobian_num;
uint8 speed_ave;
uint16 bianbi_num;
uint8 R_max, L_min;
Site_t center_line_sd[60];

Site_t center_line[60];
//中线数组
Site_t center_line_F[60];
Site_t site1 = {10, 70};                                                        //编码器数据在LCD上显示的位置

Site_t sit_L[60];//左边线的数组
Site_t sit_R[60];//右边线的数组结构体   sit_R.x  sit_R.y
Site_t sit_L_F[60];
Site_t sit_R_F[60];

uint8 bianxin[60];
uint8_t left_lost = 0, right_lost = 0, stop_line = 0, up_line = 0;
uint8_t h_left_lost = 0, h_right_lost = 0;
uint8 tubian_num, tubian_num_H;

float  bian_k;
float   QZ_UP = 10,
        QZ_DOWN = 41;
float bian_bi, bian_ca;
extern uint8 flag_stopcar;
extern int16 S_Error;
uint8  point = 40, point_last = 40;
uint8 distence[61] = { 0, 4, 4, 4, 5, 5, 5, 5, 5, 6,
                       6, 6, 7, 7, 7, 8, 8, 8, 9, 9,
                       10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
                       15, 15, 16, 16, 17, 18, 18, 19, 19, 20,
                       20, 21, 22, 22, 23, 23, 24, 24, 25, 25,
                       26, 27, 27, 28, 29, 29, 30, 30, 31, 31, 32
                     };

uint8 distence2[61] =
{
    0, 4, 4, 4, 5, 5, 5, 5, 5, 6,
    6, 6, 7, 7, 7, 8, 8, 8, 9, 9,
    10, 10, 11, 11, 12, 12, 13, 13, 14, 14,
    15, 15, 16, 16, 17, 18, 18, 19, 19, 20,
    20, 21, 22, 22, 23, 23, 24, 24, 25, 25,
    26, 27, 27, 28, 29, 29, 30, 30, 31, 31, 32
};

void image_processing(void)                                                         //获取赛道中线
{
    h_left_lost = 0; //前五行丢线标志
    h_right_lost = 0;

    right_lost = 0; //后55行丢线标志
    left_lost = 0;

    shi_flag = 0;

    za_flag_r = 0;
    za_flag_l = 0;
    za_flag = 0;

    tubian_num = 0;
    zhangai = 0;
    tubian_num_H = 0; //统计突变的行数


    /**********以下前五行补线（算出point起始点）*********/
    for(H = 59; H > 55; H --)                                                           //采集最近5行的左右边线
    {

        sit_R[H].y = H;      //记录对应的行
        sit_L[H].y = H;
        center_line[H].y = H;//中心线的行确定

        for(W = 40; W < 80; W ++)                                                        //右边线
        {

            if((img[H][W] == 0) && (img[H][W - 1]) != 0)                                  //右边能采集到边线
            {
                sit_R[H].x = W;                     //扫到右边线，记录对应的列
                bian_line[H * 2 + 1] = W;
                break;
            }
            else                                                                      //右边采集不到边线
            {
                if((W == 79) && (img[H][W] != 0))
                {
                    sit_R[H].x = 79;                 //扫不到右边线，列==79
                    bian_line[H * 2 + 1] = sit_R[H].x;
                }
            }
        }
        for(W = 40; W > 0; W--)                                                    //左边线
        {
            if((img[H][W] == 0) && (img[H][W + 1] != 0))                              //左边能采集到边线
            {
                sit_L[H].x = W;                     //扫到左边线，记录对应的列
                bian_line[H * 2] = W;
                break;
            }
            else                                                                     //左边采集不到边线
            {
                if((W == 1) && (img[H][W] != 0))
                {

                    sit_L[H].x = 0;                    //扫不到右边线，列==0
                    bian_line[H * 2] = sit_L[H].x;
                }
            }
        }

        if((bian_line[H * 2] < 1) || (bian_line[H * 2 + 1] > 78)) //边线取到的是0或者79，也就是两边至少有一边没有扫到线时
        {

            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //两边都没有扫到边线（有可能是十字）
            {
                if(H == 59)
                    center_line[H].x = 40; //如果是组后第一行，给一个40作为中线
                else
                    center_line[H].x = center_line[H + 1].x; //跟下面行的值保持一致
            }
            else if(bian_line[H * 2] < 1) //左边线没扫到
            {
                h_left_lost = 1; //最近几行左边丢线标志置位

                if(bian_line[H * 2 + 1] - distence[H] > 0) //取到的边线距离中线偏移不大
                {
                    center_line[H].x = (bian_line[H * 2 + 1] - distence[H]); //补线
                }
                else//取到的边线偏移较大
                {
                    if(H == 59)
                        center_line[H].x = 1; //如果是组后第一行，给一个1作为中线
                    else
                        center_line[H].x = center_line[H + 1].x; //跟下面行的值保持一致
                }
            }
            else if(bian_line[H * 2 + 1] > 78) //右边线没扫到
            {
                h_right_lost = 1; //最近几行右边丢线标志置位

                if((bian_line[H * 2] + distence[H]) < 79) //取到的边线距离中线偏移不大
                {
                    center_line[H].x = (bian_line[H * 2] + distence[H]); //补线
                }
                else//取到的边线偏移较大
                {
                    if(H == 59)
                        center_line[H].x = 78; //如果是组后第一行，给一个78作为中线
                    else
                        center_line[H].x = center_line[H + 1].x; //跟下面行的值保持一致
                }
            }
        }
        else      //两边均能找到边线
        {
            center_line[H].x = ((bian_line[H * 2] + bian_line[H * 2 + 1]) / 2); //左右直接取平均值
            if((bian_line[H * 2 + 1] - bian_line[H * 2]) < 45) //左右边线距离很小，车前方有障碍物
            {
                zhangai = 1;
            }
        }
    }
    /**********以上前五行补线（算出point起始点）*********/




    /***********以下是后55行识别+补线（算出centre_ave）**************/

    point = center_line[56].x; //由前五行的扫线的结果确定第55行的开始扫线点

    for(H = 55; H > 0; H --)
    {

        sit_L[H].y = H;
        sit_R[H].y = H;
        center_line[H].y = H;

        if(point != 0) //防止死机；朱齐：死机的时候point=0，改下
        {
            if(img[H][point] == 0) //如果选择的起始点本身是黑色
            {
                if(H > 40) //如果这种情况还发生在前面的行中，判定是停车线
                {
                    point = center_line[H + 2].x ; //如果在停车判断区中  不停止扫描  坐标点用上上次的值
                }
                else//如果这种情况出现在后面的线，说明出赛道，后面的线的参数都给固定值
                {
                    for(i = H; i > 0; i--)
                    {
                        center_line[i].x = 0; //中间数组为0
                        bian_line[i * 2] = 0; //左边线为0
                        bian_line[i * 2 + 1] = 79; //右边线为79
                        sit_R[i].x = 79;
                        sit_L[i].x = 0;
                    }
                    point = center_line[56].x; //把参考点返回到55行选的点
                    break;
                }
            }
        }



        /*以下是障碍处理*/
        if(za_flag == 1)
        {
            for(i = H + 8; i > (H - 5); i--) // i = H+8  H+7 H+6 H+5 H+4 H+3 H+2  H+1 H H-1 H-2 H-3 H-4
            {
                if(za_flag_r == 1) //障碍物在右
                {
                    center_line[i].x = (uint8)(center_line[i + 1].x - 1);
                    if(center_line[i].x < 1)
                    {
                        center_line[i].x = 1;
                    }
                }
                if(za_flag_l == 1) //障碍物在左
                {
                    center_line[i].x = (uint8)(center_line[i + 1].x + 1);
                    if(center_line[i].x > 78)
                    {
                        center_line[i].x = 78;
                    }
                }
                bian_line[i * 2] = 0;
                bian_line[i * 2 + 1] = 79;
                sit_R[i].x = 79;
                sit_L[i].x = 0;
            }
            for(i = (H - 5); i > 0; i--) //H-5 -----> 0
            {
                center_line[i].x = 0;
                bian_line[i * 2] = 0;
                bian_line[i * 2 + 1] = 79;
                sit_R[i].x = 79;
                sit_L[i].x = 0;
            }
            point = center_line[56].x;
            break;
        }
        /*以上是障碍处理*/


        /*以下是非十字扫边*/
        if(shi_flag == 0) //初始值即为0
        {
            if( right_lost == 0 ) //右边没有丢线
            {
                for(W = point; W < 80; W++) //从point开始向右扫线
                {
                    if((img[H][W] == 0) && (img[H][W - 1]) != 0) // 从白到黑，扫到右边线
                    {
                        sit_R[H].x = W;
                        bian_line[H * 2 + 1] = W;
                        break;
                    }
                    else
                    {
                        if((W == 79) && (img[H][W] != 0)) //如果扫到头了并且为末点为白的
                        {
                            sit_R[H].x = 79; //给一个79，右边急转
                            bian_line[H * 2 + 1] = sit_R[H].x;
                            break;
                        }
                    }
                }
                if(left_lost == 1)
                {
                    if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3] > 0) && (h_left_lost == 0)) //前面无丢线  右边拓宽 左边跳变
                    {
                        sit_R[H].x = 79;              //给右边一个79
                        bian_line[H * 2 + 1] = sit_R[H].x;

                        shi_flag = 1; //十字标志置位
                    }
                }
            }
            else//右边丢线，直接给79
            {
                sit_R[H].x = 79;
                bian_line[H * 2 + 1] = sit_R[H].x;
            }
            if( left_lost == 0 )
            {
                for(W = point; W > 0; W--) //从point开始向左扫线
                {
                    if((img[H][W] == 0) && (img[H][W + 1]) != 0)
                    {
                        sit_L[H].x = W;
                        bian_line[H * 2] = W;
                        break;
                    }
                    else
                    {
                        if((W == 1) && (img[H][W] != 0))
                        {
                            sit_L[H].x = 0;
                            bian_line[H * 2] = sit_L[H].x;
                            break;
                        }
                    }
                }
                if(right_lost == 1)
                {
                    if((bian_line[H * 2] - bian_line[H * 2 + 2] < 0) && (h_right_lost == 0)) //前面无丢线  右边跳变
                    {
                        sit_L[H].x = 0;
                        bian_line[H * 2] = sit_L[H].x;
                        shi_flag = 1;
                    }
                }
            }
            else
            {
                sit_L[H].x = 0;
                bian_line[H * 2] = sit_L[H].x;
            }

            /*巡线结束*/

            if(shi_flag == 0) //未发现十字
            {
                /*检测是否有障碍*/
                if(((bian_line[H * 2] - bian_line[H * 2 + 4]) > 10) || ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) < -10)) //有很大的边缘跳变处理
                {
                    if((h_left_lost == 0) || (h_right_lost == 0)) //前五行无全丢线
                    {
                        if(((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) < -10) && ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > -40) && (bian_line[H * 2 + 5] != 79)) //右障碍突变（判断上一次的点是不是在边缘）
                        {
                            tubian_num = bian_line[H * 2 + 5] - bian_line[H * 2 + 1];
                            tubian_num_H = H;
                            za_flag_r = 1;
                        }
                        if(((bian_line[H * 2] - bian_line[H * 2 + 4]) > 10) && ((bian_line[H * 2] - bian_line[H * 2 + 4]) < 40) && (bian_line[H * 2 + 4] != 0)) //左障碍突变（判断上一次的点是不是在边缘）
                        {
                            tubian_num = bian_line[H * 2] - bian_line[H * 2 + 4];
                            tubian_num_H = H;
                            za_flag_l = 1;
                        }
                        if(((bian_line[H * 2] - bian_line[H * 2 + 4]) > 8) && ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) < -8))
                        {
                            za_flag_r = 0;
                            za_flag_l = 0;
                            tubian_num = 0;
                            tubian_num_H = 0;
                        }
                    }
                }
                if(((bian_line[H * 2] - bian_line[H * 2 + 4]) < -6) || ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 6)) //有较大的边缘跳变处理
                {
                    if((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 6)
                    {
                        if((za_flag_r == 1) && (H > 15)) //判断是否为障碍
                        {
                            if(tubian_num - (bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 1) //判断上次突变与这次的差距大小
                            {
                                if(bian_line[H * 2 + 1] != 79) //判断跳变到的边不是边缘
                                {
                                    if(tubian_num_H - H > 3) //确定突变行数
                                    {
                                        za_flag = 1; //确定找到障碍
                                    }
                                }
                            }
                        }
                        if(H < 45) //45行后做突变补线处理
                        {
                            center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + 2;
                        }
                        else
                        {
                            center_line[H].x = center_line[H + 1].x;
                        }
                        if(zhangai == 0)
                        {
                            right_lost = 1;
                        }
                    }
                    if((bian_line[H * 2] - bian_line[H * 2 + 4]) < -6)
                    {
                        if((za_flag_l == 1) && (H > 15))
                        {
                            if(tubian_num - (bian_line[H * 2 + 4] - bian_line[H * 2]) > 1) //判断上次突变与这次的差距大小
                            {
                                if(bian_line[H * 2] != 0) //判断跳变到的边不是边缘
                                {
                                    if(tubian_num_H - H > 3) //确定突变行数
                                    {
                                        za_flag = 1; //确定找到障碍
                                    }
                                }
                            }
                        }
                        if(H < 45) //45行后做突变补线处理
                        {
                            center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - 2;
                        }
                        else
                        {
                            center_line[H].x = center_line[H + 1].x;
                        }
                        if(zhangai == 0)
                        {
                            left_lost = 1;
                        }
                    }
                    if(((bian_line[H * 2] - bian_line[H * 2 + 4]) < -5) && ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 5))
                        center_line[H].x = center_line[H + 1].x; //防起跑线误判
                }
                else  //正常边线处理（一边丢线或者不丢线）
                {
                    if(((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79)) || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))) //一边丢线
                    {
                        if(bian_line[H * 2] < 1)              // 左边一边丢线
                        {
                            if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //左丢线，右往左判断
                            {
                                if(H < 45)
                                {
                                    shi_flag = 1;
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //消掉近5行的中线
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x ;
                            }
                            else
                            {
                                if(H > 50)                    //前十行判断是否需要往中间补
                                {
                                    if(h_left_lost == 1)        //前五行有丢线，往中间补。没有往两边补
                                    {
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) + distence[H + 1] - distence[H];
                                    }
                                    else
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence[H + 1] + distence[H];
                                }
                                else
                                    center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence[H + 1] + distence[H];
                            }
                        }
                        if(bian_line[H * 2 + 1] > 78)         // 右一边丢线
                        {

                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    shi_flag = 1;
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //消掉近5行的中线
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x ;
                            }
                            else
                            {
                                if(H > 50)                        //前十行判断是否需要往中间补
                                {
                                    if(h_right_lost == 1)         //前五行有丢线，往中间补。没有往两边补
                                    {
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + distence[H] - distence[H + 1];
                                    }
                                    else
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence[H] + distence[H + 1];
                                }
                                else
                                    center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence[H] + distence[H + 1];
                            }
                        }

                    }
                    else
                    {
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79))            //两边均能找到
                        {
                            center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2;
                            //   right_lost=0;
                            //  left_lost=0;
                        }
                        else
                        {
                            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78))
                            {
                                // if(H<50)
                                shi_flag = 1;
                                for(l = 0; l < 5; l++)
                                {
                                    center_line[H + l].x = 0; //消掉近5行的中线
                                }
                                center_line[H].x = center_line[H + 1].x;
                            }
                            else
                                center_line[H].x = center_line[H + 1].x;
                        }
                    }
                }
                if((left_lost == 1) && (right_lost == 1)) //两边都没扫到判定是 十字
                {
                    shi_flag = 1;

                    for(l = 0; l < 5; l++)
                    {
                        center_line[H + l].x = 0; //消掉近5行的中线
                    }
                }
                point = center_line[H].x; //动态取点


                if(shi_flag == 1)
                {
                    if(H == 55)
                        point = 40;
                    else
                        point = center_line[H + 5].x; //取下面的值作为开始扫描点
                }
            }
            else   //十字时  处理线
            {
                point = center_line[H + 3].x;
                for(l = 0; l < 6; l++)
                {
                    center_line[H + l].x = 0; //消掉近5行的中线
                }
            }
        }
        /*以上是非十字扫边*/


        /*以下是确定是十字后的处理*/
        else
        {

            for(W = point; W < 80; W++)
            {
                if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                {
                    sit_R[H].x = W;
                    bian_line[H * 2 + 1] = W;
                    break;
                }
                else
                {
                    if((W == 79) && (img[H][W] != 0))
                    {
                        sit_R[H].x = 79;
                        bian_line[H * 2 + 1] = sit_R[H].x;
                        break;
                    }
                }
            }
            for(W = point; W > 0; W--)
            {
                if((img[H][W] == 0) && (img[H][W + 1]) != 0)
                {
                    sit_L[H].x = W;
                    bian_line[H * 2] = W;
                    break;
                }
                else
                {
                    if((W == 1) && (img[H][W] != 0))
                    {
                        sit_L[H].x = 0;
                        bian_line[H * 2] = sit_L[H].x;
                        break;
                    }
                }
            }
            if((bian_line[H * 2] > 1) && (bian_line[H * 2 + 1] < 79)) //找到两边了
            {
                if((bian_line[H * 2 + 1] - bian_line[H * 2] - distence[H] * 2) < 10) //确定十字上边的边了
                {
                    center_line[H].x = (bian_line[H * 2 + 1] + bian_line[H * 2]) / 2;
                    right_lost = 0;
                    left_lost = 0;
                    shi_flag = 0;
                }
                else
                {
                    sit_L[H].x = 0;
                    bian_line[H * 2] = sit_L[H].x;
                    sit_R[H].x = 79;
                    bian_line[H * 2 + 1] = sit_R[H].x;
                    center_line[H].x = 0;
                }

            }
            else
            {
                sit_L[H].x = 0;
                bian_line[H * 2] = sit_L[H].x;
                sit_R[H].x = 79;
                bian_line[H * 2 + 1] = sit_R[H].x;
                center_line[H].x = 0;
            }
        }
        /*以上是确定是十字后的处理*/

        ///////////////////////////////////////////////////////////////

        if(H == 1)
        {
            point = center_line[56].x;
        }


        /*以下确定是环岛后的进环处理*/
        if (Ring_Find_Flag == 1)
        {
            /*环岛在左边的情况*/
            if (LeftRing_Find_Flag == 1)
            {
                /*以下是进环操作*/
                if (h_left_lost == 0 && h_right_lost == 0 && assist0 == 1)
                {
                    assist_ADD = 1;
                    for (W = point; W < 80; W++)
                    {
                        sit_R[H].y = H;
                        if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                        {
                            sit_R[H].x = W;
                            bian_line[H * 2 + 1] = W;
                            break;
                        }
                        else
                        {
                            if((W == 79) && (img[H][W] != 0))
                            {
                                sit_R[H].x = 79;
                                bian_line[H * 2 + 1] = sit_R[H].x;
                                break;
                            }
                        }
                    }

                    center_line[H].x = bian_line[H * 2 + 1] - distence2[H] + 1;
                }

                if (assist_ADD == 1 && h_left_lost == 1 && h_right_lost == 0 )
                {
                    assist1 = 1;
                    assist0 = 0;
                    for (W = point; W < 80; W++)
                    {
                        sit_R[H].y = H;
                        if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                        {
                            sit_R[H].x = W;
                            bian_line[H * 2 + 1] = W;
                            break;
                        }
                        else
                        {
                            if((W == 79) && (img[H][W] != 0))
                            {
                                sit_R[H].x = 79;
                                bian_line[H * 2 + 1] = sit_R[H].x;
                                break;
                            }
                        }
                    }

                    center_line[H].x = bian_line[H * 2 + 1] - distence2[H] + 1;
                }

                if (h_left_lost == 0 && assist1 == 1)
                {
                    assist_ADD = 0;
                    for(W = point; W > 0; W--)
                    {

                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                        {
                            sit_L[H].x = W;
                            bian_line[H * 2] = W;
                            break;
                        }
                        else
                        {
                            if((W == 1) && (img[H][W] != 0))
                            {
                                sit_L[H].x = 0;
                                bian_line[H * 2] = sit_L[H].x;
                            }
                        }
                    }

                    center_line[H].x = bian_line[H * 2] + distence2[H] + 3;

                    if (h_left_lost == 0 && h_right_lost == 1)
                    {
                        assist2 = 1;
                    }
                }

                if(assist2 == 1 && h_left_lost == 0 && h_right_lost == 0)
                {
                    assist1 = 0;
                    assist3 = 1;
                }

                /*在环内的巡线补线操作*/
                if (assist3 == 1)
                {
                    assist2 = 0;
                    for(W = point; W > 0; W--) //向左扫描
                    {

                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                        {
                            sit_L[H].x = W;
                            bian_line[H * 2] = W;
                            break;
                        }
                        else
                        {
                            if((W == 1) && (img[H][W] != 0))
                            {
                                sit_L[H].x = 0;
                                bian_line[H * 2] = sit_L[H].x;
                            }
                        }
                    }

                    for (W = point; W < 80; W++) //向右扫描
                    {
                        sit_R[H].y = H;
                        if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                        {
                            sit_R[H].x = W;
                            bian_line[H * 2 + 1] = W;
                            break;
                        }
                        else
                        {
                            if((W == 79) && (img[H][W] != 0))
                            {
                                sit_R[H].x = 79;
                                bian_line[H * 2 + 1] = sit_R[H].x;
                                break;
                            }
                        }
                    }

                    if(
                        ((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79))
                        || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))
                    )
                    {
                        if(bian_line[H * 2] < 1)
                        {
                            if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0;
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x + 5;
                            }

                            else
                            {
                                if(H > 50)
                                {
                                    if(h_left_lost == 1)
                                    {
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) + distence2[H + 1] - distence2[H] + 5;
                                    }
                                    else
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;
                                }
                                else
                                    center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;
                            }
                        }

                        if(bian_line[H * 2 + 1] > 78) //右边丢线
                        {
                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //消掉近5行的中线
                                    }
                                }

                                else
                                    center_line[H].x = center_line[H + 1].x + 5;
                            }

                            else
                            {
                                if(H > 50)                          //前十行判断是否需要往中间补
                                {
                                    if(h_right_lost == 1)           //前五行有丢线，往中间补。没有往两边补
                                    {
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + distence2[H] - distence2[H + 1] + 5;
                                    }
                                    else
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 5;
                                }

                                else
                                    center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 5;
                            }
                        }
                    }

                    else
                    {
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //双边都不丢线
                        {
                            center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 5;
                        }

                        else
                        {
                            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //在这里，真的检测到出环口了，
                            {
                                assist4 = 1;
                                assist3 = 0;
                                //led(LED0, LED_ON);
                            }
                            else
                                center_line[H].x = center_line[H + 1].x + 5;
                        }
                    }
                }
            }

            /*环岛在右边的情况*/
            if (RightRing_Find_Flag == 1)
            {
                /*以下是进环操作*/
                if (h_right_lost == 0 && h_left_lost == 0 && assistr0 == 1) //前55行两边都不丢线
                {
                    assist_rADD = 1;
                    for(W = point; W > 0; W--) //向左扫描
                    {

                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                        {
                            sit_L[H].x = W;
                            bian_line[H * 2] = W;
                            break;
                        }
                        else
                        {
                            if((W == 1) && (img[H][W] != 0))
                            {
                                sit_L[H].x = 0;
                                bian_line[H * 2] = sit_L[H].x;
                            }
                        }
                    }

                    center_line[H].x = bian_line[H * 2] + distence2[H] + 1;

                }

                if (assist_rADD == 1 && h_right_lost == 1 && h_left_lost == 0 )  //前55行右边丢线，左边没丢
                {
                    assistr1 = 1;
                    assistr0 = 0;
                    for(W = point; W > 0; W--) //向左扫描
                    {

                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                        {
                            sit_L[H].x = W;
                            bian_line[H * 2] = W;
                            break;
                        }
                        else
                        {
                            if((W == 1) && (img[H][W] != 0))
                            {
                                sit_L[H].x = 0;
                                bian_line[H * 2] = sit_L[H].x;
                            }
                        }
                    }
                    center_line[H].x = bian_line[H * 2] + distence2[H] + 1;
                }

                if (h_right_lost == 0 && assistr1 == 1)//前55行右边丢线
                {
                    assist_rADD = 0;
                    for (W = point; W < 80; W++) //向右扫描
                    {
                        sit_R[H].y = H;
                        if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                        {
                            sit_R[H].x = W;
                            bian_line[H * 2 + 1] = W;
                            break;
                        }
                        else
                        {
                            if((W == 79) && (img[H][W] != 0))
                            {
                                sit_R[H].x = 79;
                                bian_line[H * 2 + 1] = sit_R[H].x;
                                break;
                            }
                        }
                    }

                    center_line[H].x = bian_line[H * 2 + 1] - distence2[H] + 1;

                    if (h_right_lost == 0 && h_left_lost == 1)
                    {
                        assistr2 = 1;
                    }
                }

                if(assistr2 == 1 && h_right_lost == 0 && h_left_lost == 0) //两边都没丢线
                {
                    assistr1 = 0;
                    assistr3 = 1;
                }

                /*在环内的巡线补线操作*/
                if (assistr3 == 1)
                {
                    assistr2 = 0;
                    for(W = point; W > 0; W--) //向左扫描
                    {

                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                        {
                            sit_L[H].x = W;
                            bian_line[H * 2] = W;
                            break;
                        }
                        else
                        {
                            if((W == 1) && (img[H][W] != 0))
                            {
                                sit_L[H].x = 0;
                                bian_line[H * 2] = sit_L[H].x;
                            }
                        }
                    }

                    for (W = point; W < 80; W++) //向右扫描
                    {
                        sit_R[H].y = H;
                        if((img[H][W] == 0) && (img[H][W - 1]) != 0)
                        {
                            sit_R[H].x = W;
                            bian_line[H * 2 + 1] = W;
                            break;
                        }
                        else
                        {
                            if((W == 79) && (img[H][W] != 0))
                            {
                                sit_R[H].x = 79;
                                bian_line[H * 2 + 1] = sit_R[H].x;
                                break;
                            }
                        }
                    }

                    if(
                        ((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79))
                        || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))
                    )
                    {
                        if(bian_line[H * 2] < 1)
                        {
                            if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0;
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x;
                            }

                            else
                            {
                                if(H > 50)
                                {
                                    if(h_left_lost == 1)
                                    {
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) + distence2[H + 1] - distence2[H];
                                    }
                                    else
                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H];
                                }
                                else
                                    center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H];
                            }
                        }

                        if(bian_line[H * 2 + 1] > 78) //右边丢线
                        {
                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //消掉近5行的中线
                                    }
                                }

                                else
                                    center_line[H].x = center_line[H + 1].x;
                            }

                            else
                            {
                                if(H > 50)                          //前十行判断是否需要往中间补
                                {
                                    if(h_right_lost == 1)           //前五行有丢线，往中间补。没有往两边补
                                    {
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + distence2[H] - distence2[H + 1];
                                    }
                                    else
                                        center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1];
                                }

                                else
                                    center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1];
                            }
                        }
                    }

                    else
                    {
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //双边都不丢线
                        {
                            center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2;
                        }

                        else
                        {
                            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //在这里，真的检测到出环口了，
                            {
                                assistr4 = 1;
                                assistr3 = 0;
                            }
                            else
                                center_line[H].x = center_line[H + 1].x;
                        }
                    }
                }
            }
        }

                            /*以下为左环岛出去的操作*/
                            if (assist4 == 1) 
                            { 
                                          shi_flag = 0; 
                                          for(W = point; W > 0; W--) 
                                          { 
                                                       sit_L[H].y = H; 
                                                       if((img[H][W] == 0) && (img[H][W + 1] != 0))
                                                       { 
                                                                    sit_L[H].x = W;
                                                                    bian_line[H * 2] = W;
                                                                    break;
                                                       } 
                                                       else
                                                       { 
                                                                     if((W == 1) && (img[H][W] != 0))
                                                                     { 
                                                                                 sit_L[H].x = 0;
                                                                                 bian_line[H * 2] = sit_L[H].x;
                                                                     } 
                                                       } 
                                          } 

                                          for (W = point; W < 80; W++)
                                          { 
                                                       sit_R[H].y = H;
                                                       if((img[H][W] == 0) && (img[H][W - 1]) != 0) 
                                                       { 
                                                                        sit_R[H].x = W; 
                                                                        bian_line[H * 2 + 1] = W; 
                                                                        break; 
                                                       } 
                                                       else 
                                                       { 
                                                                        if((W == 79) && (img[H][W] != 0)) 
                                                                        { 
                                                                                       sit_R[H].x = 79; 
                                                                                       bian_line[H * 2 + 1] = sit_R[H].x; 
                                                                                       break; 
                                                                        } 
                                                       } 
                                          } 

                                          if( 
                                                       ((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79)) 
                                                       || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))//有一边丢线
                                            ) 
                                            { 
//                                                        if(bian_line[H * 2] < 1) //左边丢线，右边不丢线
//                                                        { 
//                                                                       if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //这种情况很正常啊
//                                                                       { 
//                                                                                    if(H < 45) 
//                                                                                    { 
//                                                                                              for(l = 0; l < 5; l++) 
//                                                                                              { 
//                                                                                                       center_line[H + l].x = 0; 
//                                                                                              } 
//                                                                                     } 
//                                                                                     else 
//                                                                                              center_line[H].x = center_line[H + 1].x - 5; 
//                                                                       } 
//                                                                       
//                                                                       else
//                                                                       {
//                                                                                    
//                                                                                    if(H > 50)
//                                                                                    {
//                                                                                               if(h_left_lost == 1) 
//                                                                                               { 
//                                                                                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) + distence2[H + 1]- distence2[H] + 5;  
//                                                                                               } 
//                                                                                               else  
//                                                                                                    center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;
//                                                                                    }
//                                                                                    else
//                                                                                                center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;  
//                                                                        } 
//                                                        }

                                                        if(bian_line[H * 2 + 1] > 78)//右边丢线，左边不丢线
                                                        { 
                                                                       if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                                                                       { 
                                                                                       center_line[H].x = center_line[H + 1].x + 10;
                                                                        } 
 
                                                                       else 
                                                                       { 
                                                                                    if(H > 50) //前十行判断是否需要往中间补 
                                                                                    {
                                                                                                 if(h_right_lost == 1) //前五行有丢线，往中间补。没有往两边补
                                                                                                 {
                                                                                                          center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + distence2[H] - distence2[H + 1] + 10;  
                                                                                                 }
                                                                                                 
                                                                                                 else
                                                                                                          center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 10;
                                                                                    }
                                                                                    
                                                                                    else
                                                                                                center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 10; 
                                                                        } 
                                                        } 
                                           
                                            }
                                           else //两边都丢线 或者 没丢线
                                           { 
                                                          
                                                         if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //没丢线
                                                         {
                                                                        center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 10;
                                                         }
                                                        
                                                         else //两边都丢线
                                                         { 
                                                                        if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78))
                                                                        {
																		  
                                                                                     for(l = 0; l < 5; l++)
                                                                                     {
                                                                                                  center_line[H + l].x = center_line[H].x - 10; //消掉近5行的中线
                                                                                      } 
                                                                                      center_line[H].x = center_line[H + 1].x - 10;
                                                                        }
                                                                        else
                                                                                      center_line[H].x = center_line[H + 1].x - 10;  
                                                          }
                                            }

                                         if ( 
                                                     (    h_right_lost == 0 && right_lost == 0 && left_lost == 1 
                                                          && bian_line[H * 2 + 5] < 78 && bian_line[H * 2 + 3] < 78 
                                                          && bian_line[H * 2 + 1] < 78 
                                                     ) 
                                            ) 
                                            { 
                                                    assist5 = 1; 
                                                    assist4 = 0; 
                                            } 
                            } 
                            if (assist5 == 1) 
                            { 
                                          for (W = point; W < 80; W++)
                                          { 
                                                           sit_R[H].y = H; 
                                                           if((img[H][W] == 0) && (img[H][W - 1]) != 0)                                       
                                                           { 
                                                                        bian_line[H * 2 + 1] = W; 
                                                                        break; 
                                                           } 
                                                           else 
                                                           { 
                                                                         if((W == 79) && (img[H][W] != 0)) 
                                                                         { 
                                                                                     sit_R[H].x = 79; 
                                                                                     bian_line[H * 2 + 1] = sit_R[H].x; 
                                                                                     break; 
                                                                         } 
                                                           } 
                                          } 
                                          
                                          center_line[H].x = bian_line[H * 2 + 1] - distence2[H] + 5;

                                          if (h_right_lost == 0 && h_left_lost == 1)
                                          { 
                                                           assist5 = 0;
                                                           assist6 = 1;
                                          } 
                            } 
                            if (assist6 == 1 && h_right_lost == 0 && h_left_lost == 0) 
                            { 
                                          assist6 = 0; 
                                          Ring_Find_Flag = 0; 
                                          LeftRing_Find_Flag = 0; 
                            } 
                            /*以上为左环岛出去的操作*/ 

                            
                            /*以下为右环岛出去的操作*/ 
                            if (assistr4 == 1) 
                            { 
                                          shi_flag = 0; 
                                          for(W = point; W > 0; W--) 
                                          { 
                                                       sit_L[H].y = H; 
                                                       if((img[H][W] == 0) && (img[H][W + 1] != 0))
                                                       { 
                                                                    sit_L[H].x = W;
                                                                    bian_line[H * 2] = W;
                                                                    break;
                                                       } 
                                                       else
                                                       { 
                                                                     if((W == 1) && (img[H][W] != 0))
                                                                     { 
                                                                                 sit_L[H].x = 0;
                                                                                 bian_line[H * 2] = sit_L[H].x;
                                                                     } 
                                                       } 
                                          } 

                                          for (W = point; W < 80; W++)
                                          { 
                                                       sit_R[H].y = H;
                                                       if((img[H][W] == 0) && (img[H][W - 1]) != 0) 
                                                       { 
                                                                        sit_R[H].x = W; 
                                                                        bian_line[H * 2 + 1] = W; 
                                                                        break; 
                                                       } 
                                                       else 
                                                       { 
                                                                        if((W == 79) && (img[H][W] != 0)) 
                                                                        { 
                                                                                       sit_R[H].x = 79; 
                                                                                       bian_line[H * 2 + 1] = sit_R[H].x; 
                                                                                       break; 
                                                                        } 
                                                       } 
                                          } 

                                       
                                          if( 
                                                       ((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79)) 
                                                       || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))//有一边丢线
                                            ) 
                                            { 
                                                        if(bian_line[H * 2] < 1) //左边丢线，右边不丢线
                                                        { 
                                                                       if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //这种情况很正常啊
                                                                       { 
                                                                                    if(H < 45) 
                                                                                    { 
                                                                                              for(l = 0; l < 5; l++) 
                                                                                              { 
                                                                                                       center_line[H + l].x = 0; 
                                                                                              } 
                                                                                     } 
                                                                                     else 
                                                                                              center_line[H].x = center_line[H + 1].x - 5; 
                                                                       } 
                                                                       
                                                                       else
                                                                       {
                                                                                    
                                                                                    if(H > 50)
                                                                                    {
                                                                                               if(h_left_lost == 1) 
                                                                                               { 
                                                                                                        center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) + distence2[H + 1]- distence2[H] + 5;  
                                                                                               } 
                                                                                               else  
                                                                                                    center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;
                                                                                    }
                                                                                    else
                                                                                                center_line[H].x = bian_line[H * 2 + 1] - (bian_line[H * 2 + 3] - point) - distence2[H + 1] + distence2[H] + 5;  
                                                                        } 
                                                        }

                                                        if(bian_line[H * 2 + 1] > 78)//右边丢线，左边不丢线
                                                        { 
                                                                       if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                                                                       { 
                                                                                       center_line[H].x = center_line[H + 1].x + 10;
                                                                        } 
 
                                                                       else 
                                                                       { 
                                                                                    if(H > 50) //前十行判断是否需要往中间补 
                                                                                    {
                                                                                                 if(h_right_lost == 1) //前五行有丢线，往中间补。没有往两边补
                                                                                                 {
                                                                                                          center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] + distence2[H] - distence2[H + 1] + 10;  
                                                                                                 }
                                                                                                 
                                                                                                 else
                                                                                                          center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 10;
                                                                                    }
                                                                                    
                                                                                    else
                                                                                                center_line[H].x = point - bian_line[H * 2 + 2] + bian_line[H * 2] - distence2[H] + distence2[H + 1] + 10; 
                                                                        } 
                                                        } 
                                           
                                            }
                                           else //两边都丢线 或者 没丢线
                                           { 
                                                          
                                                         if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //没丢线
                                                         {
                                                                        center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 10;
                                                         }
                                                        
                                                         else //两边都丢线
                                                         { 
                                                                        if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78))
                                                                        {
																		  
                                                                                     for(l = 0; l < 5; l++)
                                                                                     {
                                                                                                  center_line[H + l].x = 75; //消掉近5行的中线
                                                                                      } 
                                                                                      center_line[H].x = 75;
                                                                        }
                                                                        else
                                                                                      center_line[H].x = 75;  
																		
																	
                                                          }
                                            }

										  
					         if(img[59][0]==255// && img[59][79]==255
								 &&img[58][0]==255 //&& img[58][79]==255
								  && img[54][0]==255 //&& img[57][79]==255
									&& img[57][0]==255// && img[56][79]==255
									 &&  img[56][0]==255 //&& img[55][79]==255
										&& img[55][0]==255 //&& img[54][79]==255
										  && img[53][0]==255 //&& img[53][79]==255
											&& img[52][0]==255 
											  && img[51][0]==255
												&& img[50][0]==255
												  && img[49][0]==255
													&& img[48][0]==255
													  && img[47][0]==255
														&& img[46][0]==255
														  && img[45][0]==255
															&& img[44][0]==255
															  && img[43][0]==255
																&& img[42][0]==255)
							  {
								center_line[H].x = 60;
								zhuqi_test = 1;
							  }
							  
								
							  
                                       if (          zhuqi_test == 1&& img[48][0]==0
                                                    // h_left_lost == 0 &&
										             //left_lost == 0 && right_lost == 0
                                                    //&& bian_line[H * 2 + 4] < 78 && bian_line[H * 2 + 2] < 78 
                                                   // && bian_line[H * 2] < 78

										   
                                          ) 
                                          { 
											        zhuqi_test = 0;
                                                    assistr5 = 1;
                                                    assistr4 = 0;
                                          } 

                            } 
                            if (assistr5 == 1)
                            { 

								 
							  
					
								         for(W = point; W > 0; W--) 
                                          { 
                                                        sit_L[H].y = H; 
                                                        if((img[H][W] == 0) && (img[H][W + 1] != 0))
                                                        { 
                                                                         sit_L[H].x = W;
                                                                         bian_line[H * 2] = W;
                                                                         break;
                                                        } 
                                                        else
                                                        {
                                                                         if((W == 1) && (img[H][W] != 0))
                                                                         {
                                                                                    sit_L[H].x = 0;
                                                                                    bian_line[H * 2] = sit_L[H].x;
                                                                          } 
                                                         } 
                                          } 
                                         
										  center_line[H].x = bian_line[H * 2] + distence2[H] + 3;

                                          if (h_left_lost == 0 && h_right_lost == 1)
                                          { 
                                          assistr5 = 0; 
                                          Ring_Find_Flag = 0; 
                                          RightRing_Find_Flag = 0;
                                          } 
							  }
							




    }


    /*以下是环岛检测*/

        if (Ring_Find_Flag == 0)
        {
            for(Ring_Line_Value = 48; Ring_Line_Value > 28; Ring_Line_Value-- ) //48~29行
            {
                if(
                    h_left_lost == 0 && h_right_lost == 0 && left_lost == 1 && right_lost == 0

                    && (bian_line[Ring_Line_Value * 2] < 1)
                    && (bian_line[Ring_Line_Value * 2 + 6] >  1)

                    && (bian_line[Ring_Line_Value * 2 - 3] < 78)
                    && (bian_line[Ring_Line_Value * 2 - 5] < 78)
                    && (bian_line[Ring_Line_Value * 2 - 7] < 78)
                    && (bian_line[Ring_Line_Value * 2 + 1] < 78)
                    && (bian_line[Ring_Line_Value * 2 + 7] < 78)

                    && ((bian_line[Ring_Line_Value * 2 + 6] - bian_line[Ring_Line_Value * 2]) > 20)
                    && ((bian_line[Ring_Line_Value * 2 + 7] - bian_line[Ring_Line_Value * 2 + 1]) < 3)
                )
                {
                    Ring_Find_Flag = 1;
                    LeftRing_Find_Flag = 1;
                    assist0 = 1;

                }//左环岛识别

                    zhuqi_2 = bian_line[Ring_Line_Value * 2 + 1] - bian_line[Ring_Line_Value * 2 + 7];
					zhuqi_3 = bian_line[Ring_Line_Value * 2 + 6] - bian_line[Ring_Line_Value * 2];
                if(
                    h_right_lost == 0 && h_left_lost == 0 //前55行两边都丢线
                    && right_lost == 1 && left_lost == 0//后面的右边丢线，左边不丢线

                    && (bian_line[Ring_Line_Value * 2] > 1)    //第一行左边线没丢线
                    && (bian_line[Ring_Line_Value * 2 + 2] > 1)//第二行左边线没丢线
                    && (bian_line[Ring_Line_Value * 2 + 4] > 1)//第三行左边线没丢线
                    && (bian_line[Ring_Line_Value * 2 + 6] > 1)//第四行左边线没丢线
                    && (bian_line[Ring_Line_Value * 2 + 8] > 1)//第五行左边线没丢线

					 && (bian_line[Ring_Line_Value * 2 + 1] > 78) //第一行右边线丢线
					 && (bian_line[Ring_Line_Value * 2 + 5] < 78) //第三行右边线没丢线



                    && ((bian_line[Ring_Line_Value * 2 + 1] - bian_line[Ring_Line_Value * 2 + 7]) > 5)//右边第一行与第四行差大于20
                    && ((bian_line[Ring_Line_Value * 2 + 6] - bian_line[Ring_Line_Value * 2]) < 20)//左边第一行与第四行差小于3
					  

                )
                {
                    Ring_Find_Flag = 1;
                    RightRing_Find_Flag = 1;
                    assistr0 = 1;
                }//右环岛识别
            }
        }






    /********加权算偏移量*********/
    for(i = 0; i < 60; i ++)                                                          //（给舵机）中线加权平均值计算
    {
        if((center_line[i].x > 1) && (center_line[i].x < 78)) //只计算从2~77的
        {
            sum_center += center_line[i].x;//加权
            num_ave ++;//加的次数
            if((i > (int)QZ_UP) && (i < (int)QZ_DOWN))    //处于在10~40行，非常有效的行
            {
                sum_center += center_line[i].x * 3; //一个数字当三个数加
                num_ave += 3;
            }
        }
    }

    ave_center = sum_center / num_ave; //求平均值
    sum_center = 0; //中心值和 归位
    num_ave = 0; //加的次数归位
    /*
    bianbi_num=0;
    R_max=40;
    L_min=40;

    for(i=0;i<60;i++)                                                            //（给电机）中线加权平均值计算
    {
    if((center_line[i].x>0)&&(center_line[i].x<79))                               //检测中线是否有效（1~78）
    {
      if(center_line[i].x>R_max)     //找到右边线最大值
      {
        R_max=center_line[i].x;
      }
      if(center_line[i].x<L_min)     //找到左边线最小值
      {
        L_min=center_line[i].x;
      }

      if(num_ave==0)
      {
        speed_ave=i;                    //扫描到第一行有效值得时候  记录下来
       // bian_bi=(float)(60-speed_ave)/60.0;    //计算出扫描到线所占的比例
      }
      sum_center+=center_line[i].x;
      num_ave++;
      if((i>speed_ave)&&(i<speed_ave+(60-speed_ave)/2))     //只算前一半线的前瞻    //加权
      {
        sum_center+=center_line[i].x;
        sum_center+=center_line[i].x;

        num_ave+=2;
      }
      bianbi_num++;
    }
    }
    bian_bi=(float)bianbi_num/60.0;

    if((1-(float)(R_max-L_min)/72.0)>0)
    {
    bian_ca=1-((float)(R_max-L_min)/72.0);
    }
    else
    bian_ca=0;

    ave_center_s=sum_center/num_ave;
    sum_center=0;
    num_ave=0;

    for(i=0;i<60;i++)
    {
      line[i]=center_line[i].x;
    }
    for(i=0;i<120;i++)
    {
      bian[i]=bian_line[i];
    }

     for(i=0;i<60;i++)                                                              //中线加权平均值计算
     {
       sit_L_F[i].x=sit_L[i].x;
       sit_R_F[i].x=sit_R[i].x;
       center_line_F[i].x=center_line[i].x;

       sit_L_F[i].y=sit_L[i].y+68;
       sit_R_F[i].y=sit_R[i].y+68;
       center_line_F[i].y=center_line[i].y+68;
     }
    for(i=0;i<60;i++)    //求边心数组
    {
      bianxin[i]=(sit_R[i].x-sit_L[i].x)/2;
    }

    S_num=0;
    ave_S_num=0;

    for(i=59;i>40;i--)
    {
       S_num+=center_line[i].x;
    }
    ave_S_num=S_num/19;
    */

}
void dir_control(void)
{
    dir_zhongzhi = 4413;

    center = (uint16)dir_zhongzhi; //舵机中值
    current_error = ave_center - 40; //与中心的差值

    dir_kp=(uint16)(dir_kp1+(current_error*current_error)/20); //动态P值  		dir_kp1=1.6, dir_kp2=60,dir_kd=8;

   dir_kp = 60;
	dir_kd = 60;


  /*  if(ave_center > 45 || ave_center < 35)
    {
        dir_kp = 95;
    }
    if(ave_center > 50 || ave_center < 30)
    {
        dir_kp = 100;
    }
    if(ave_center > 58 || ave_center < 20)
    {
        dir_kp = 100;
    }
*/



    dir_out = (center + dir_kp * current_error + dir_kd * (current_error - last_error)); //PID算法

    if(dir_out < 0)dir_out = 0;

    dir = dir_out;

    if(dir > (center + 1033)) //限幅
    {
        dir = (center + 1033);
    }
    if(dir < (center - 1033))
    {
        dir = (center - 1033);
    }

    last_error = current_error; //记录上次的差值
    last_dir = dir; //记录上次给的舵机值

    //dir_pianjiao=dir+100-(uint16)dir_zhongzhi;//这句感觉没什么用

     ftm_pwm_duty(FTM1, FTM_CH0,(uint16)dir);  // 使能舵机 
   // ftm_pwm_duty(FTM1, FTM_CH0,5446);  // 测试舵机   4413  3380   5446


}

void img_extract(void *dst, void *src, uint32_t srclen)
{
    uint8_t colour[2] = {255, 0}; //0 和 1 分别对应的颜色
    uint8_t *mdst = dst;
    uint8_t *msrc = src;
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8_t tmpsrc;
    while(srclen --)
    {
        tmpsrc = *msrc++;
        *mdst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *mdst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}