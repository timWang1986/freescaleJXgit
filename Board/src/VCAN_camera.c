#include "common.h"
#include "include.h"
#include "VCAN_camera.h"

float dir_zhongzhi;
uint16 center;
uint16  last_error,                                                             //�ϴε�ƫ��
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
uint8  Ring_Line_Value = 0; //����ʶ���־λ

uint8  assist0 = 0;
uint8  assist1 = 0;//
uint8  assist2 = 0;//
uint8  assist3 = 0;
uint8  assist4 = 0;
uint8  assist5 = 0;
uint8  assist6 = 0;
uint8  assist_ADD = 0;/////////�󻷵���־λ

uint8  assistr0 = 0;
uint8  assistr1 = 0;//�뻷��
uint8  assistr2 = 0;//���߶�û����
uint8  assistr3 = 0;//����Ѳ��
uint8  assistr4 = 0;//������
uint8  assistr5 = 0;
uint8  assistr6 = 0;
uint8  assist_rADD = 0;/////////�һ�����־λ


uint8  car_stop_flag = 0, podao_flag = 0;
uint8  zhangai = 0;
uint8  obstruct;
uint8  bian0_L;
uint8  bian0_R;
uint8  distence1[60];
uint8  bian_distence[60];
uint8  imgbuff[CAMERA_SIZE];                                                    //����洢����ͼ�������
uint8  img[60][CAMERA_W];                                                 //������ͼ������
uint8  H, W;                                                                    //ͼ�������С�����
uint8  i, j, k, l;                                                                //Ӧ�ñ���
uint8  bian_line[120];                                                          //�������飨����Ϊ��ż��Ϊ�ң�        //����ͣ��               //��ȡ�ı���������
uint16  ave_center = 40;                                                            //���ߵļ�Ȩƽ��ֵ
uint16  ave_S_num, S_num;
uint16  ave_center_s = 40;
uint8  last_ave_center;                                                         //��һ�ε����߼�Ȩƽ��ֵ
uint16 sum_center;                                                              //���ߵ��ܺ�             //��ǰƫ��
int16  error_bian_L[2];
int16  error_bian_R[2];
int16  error2_bian_L;
int16  error2_bian_R;
uint8  bian_flag_R;
uint8  bian_flag_L;
uint8  za_flag_r, za_flag_l, za_flag;
uint8  flag_R;
uint8  flag_L;
uint8  num_ave = 0;                                                             //��Ч���߸��������ڼ������߼�Ȩƽ��ֵ
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
//��������
Site_t center_line_F[60];
Site_t site1 = {10, 70};                                                        //������������LCD����ʾ��λ��

Site_t sit_L[60];//����ߵ�����
Site_t sit_R[60];//�ұ��ߵ�����ṹ��   sit_R.x  sit_R.y
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

void image_processing(void)                                                         //��ȡ��������
{
    h_left_lost = 0; //ǰ���ж��߱�־
    h_right_lost = 0;

    right_lost = 0; //��55�ж��߱�־
    left_lost = 0;

    shi_flag = 0;

    za_flag_r = 0;
    za_flag_l = 0;
    za_flag = 0;

    tubian_num = 0;
    zhangai = 0;
    tubian_num_H = 0; //ͳ��ͻ�������


    /**********����ǰ���в��ߣ����point��ʼ�㣩*********/
    for(H = 59; H > 55; H --)                                                           //�ɼ����5�е����ұ���
    {

        sit_R[H].y = H;      //��¼��Ӧ����
        sit_L[H].y = H;
        center_line[H].y = H;//�����ߵ���ȷ��

        for(W = 40; W < 80; W ++)                                                        //�ұ���
        {

            if((img[H][W] == 0) && (img[H][W - 1]) != 0)                                  //�ұ��ܲɼ�������
            {
                sit_R[H].x = W;                     //ɨ���ұ��ߣ���¼��Ӧ����
                bian_line[H * 2 + 1] = W;
                break;
            }
            else                                                                      //�ұ߲ɼ���������
            {
                if((W == 79) && (img[H][W] != 0))
                {
                    sit_R[H].x = 79;                 //ɨ�����ұ��ߣ���==79
                    bian_line[H * 2 + 1] = sit_R[H].x;
                }
            }
        }
        for(W = 40; W > 0; W--)                                                    //�����
        {
            if((img[H][W] == 0) && (img[H][W + 1] != 0))                              //����ܲɼ�������
            {
                sit_L[H].x = W;                     //ɨ������ߣ���¼��Ӧ����
                bian_line[H * 2] = W;
                break;
            }
            else                                                                     //��߲ɼ���������
            {
                if((W == 1) && (img[H][W] != 0))
                {

                    sit_L[H].x = 0;                    //ɨ�����ұ��ߣ���==0
                    bian_line[H * 2] = sit_L[H].x;
                }
            }
        }

        if((bian_line[H * 2] < 1) || (bian_line[H * 2 + 1] > 78)) //����ȡ������0����79��Ҳ��������������һ��û��ɨ����ʱ
        {

            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //���߶�û��ɨ�����ߣ��п�����ʮ�֣�
            {
                if(H == 59)
                    center_line[H].x = 40; //���������һ�У���һ��40��Ϊ����
                else
                    center_line[H].x = center_line[H + 1].x; //�������е�ֵ����һ��
            }
            else if(bian_line[H * 2] < 1) //�����ûɨ��
            {
                h_left_lost = 1; //���������߶��߱�־��λ

                if(bian_line[H * 2 + 1] - distence[H] > 0) //ȡ���ı��߾�������ƫ�Ʋ���
                {
                    center_line[H].x = (bian_line[H * 2 + 1] - distence[H]); //����
                }
                else//ȡ���ı���ƫ�ƽϴ�
                {
                    if(H == 59)
                        center_line[H].x = 1; //���������һ�У���һ��1��Ϊ����
                    else
                        center_line[H].x = center_line[H + 1].x; //�������е�ֵ����һ��
                }
            }
            else if(bian_line[H * 2 + 1] > 78) //�ұ���ûɨ��
            {
                h_right_lost = 1; //��������ұ߶��߱�־��λ

                if((bian_line[H * 2] + distence[H]) < 79) //ȡ���ı��߾�������ƫ�Ʋ���
                {
                    center_line[H].x = (bian_line[H * 2] + distence[H]); //����
                }
                else//ȡ���ı���ƫ�ƽϴ�
                {
                    if(H == 59)
                        center_line[H].x = 78; //���������һ�У���һ��78��Ϊ����
                    else
                        center_line[H].x = center_line[H + 1].x; //�������е�ֵ����һ��
                }
            }
        }
        else      //���߾����ҵ�����
        {
            center_line[H].x = ((bian_line[H * 2] + bian_line[H * 2 + 1]) / 2); //����ֱ��ȡƽ��ֵ
            if((bian_line[H * 2 + 1] - bian_line[H * 2]) < 45) //���ұ��߾����С����ǰ�����ϰ���
            {
                zhangai = 1;
            }
        }
    }
    /**********����ǰ���в��ߣ����point��ʼ�㣩*********/




    /***********�����Ǻ�55��ʶ��+���ߣ����centre_ave��**************/

    point = center_line[56].x; //��ǰ���е�ɨ�ߵĽ��ȷ����55�еĿ�ʼɨ�ߵ�

    for(H = 55; H > 0; H --)
    {

        sit_L[H].y = H;
        sit_R[H].y = H;
        center_line[H].y = H;

        if(point != 0) //��ֹ���������룺������ʱ��point=0������
        {
            if(img[H][point] == 0) //���ѡ�����ʼ�㱾���Ǻ�ɫ
            {
                if(H > 40) //������������������ǰ������У��ж���ͣ����
                {
                    point = center_line[H + 2].x ; //�����ͣ���ж�����  ��ֹͣɨ��  ����������ϴε�ֵ
                }
                else//���������������ں�����ߣ�˵����������������ߵĲ��������̶�ֵ
                {
                    for(i = H; i > 0; i--)
                    {
                        center_line[i].x = 0; //�м�����Ϊ0
                        bian_line[i * 2] = 0; //�����Ϊ0
                        bian_line[i * 2 + 1] = 79; //�ұ���Ϊ79
                        sit_R[i].x = 79;
                        sit_L[i].x = 0;
                    }
                    point = center_line[56].x; //�Ѳο��㷵�ص�55��ѡ�ĵ�
                    break;
                }
            }
        }



        /*�������ϰ�����*/
        if(za_flag == 1)
        {
            for(i = H + 8; i > (H - 5); i--) // i = H+8  H+7 H+6 H+5 H+4 H+3 H+2  H+1 H H-1 H-2 H-3 H-4
            {
                if(za_flag_r == 1) //�ϰ�������
                {
                    center_line[i].x = (uint8)(center_line[i + 1].x - 1);
                    if(center_line[i].x < 1)
                    {
                        center_line[i].x = 1;
                    }
                }
                if(za_flag_l == 1) //�ϰ�������
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
        /*�������ϰ�����*/


        /*�����Ƿ�ʮ��ɨ��*/
        if(shi_flag == 0) //��ʼֵ��Ϊ0
        {
            if( right_lost == 0 ) //�ұ�û�ж���
            {
                for(W = point; W < 80; W++) //��point��ʼ����ɨ��
                {
                    if((img[H][W] == 0) && (img[H][W - 1]) != 0) // �Ӱ׵��ڣ�ɨ���ұ���
                    {
                        sit_R[H].x = W;
                        bian_line[H * 2 + 1] = W;
                        break;
                    }
                    else
                    {
                        if((W == 79) && (img[H][W] != 0)) //���ɨ��ͷ�˲���Ϊĩ��Ϊ�׵�
                        {
                            sit_R[H].x = 79; //��һ��79���ұ߼�ת
                            bian_line[H * 2 + 1] = sit_R[H].x;
                            break;
                        }
                    }
                }
                if(left_lost == 1)
                {
                    if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3] > 0) && (h_left_lost == 0)) //ǰ���޶���  �ұ��ؿ� �������
                    {
                        sit_R[H].x = 79;              //���ұ�һ��79
                        bian_line[H * 2 + 1] = sit_R[H].x;

                        shi_flag = 1; //ʮ�ֱ�־��λ
                    }
                }
            }
            else//�ұ߶��ߣ�ֱ�Ӹ�79
            {
                sit_R[H].x = 79;
                bian_line[H * 2 + 1] = sit_R[H].x;
            }
            if( left_lost == 0 )
            {
                for(W = point; W > 0; W--) //��point��ʼ����ɨ��
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
                    if((bian_line[H * 2] - bian_line[H * 2 + 2] < 0) && (h_right_lost == 0)) //ǰ���޶���  �ұ�����
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

            /*Ѳ�߽���*/

            if(shi_flag == 0) //δ����ʮ��
            {
                /*����Ƿ����ϰ�*/
                if(((bian_line[H * 2] - bian_line[H * 2 + 4]) > 10) || ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) < -10)) //�кܴ�ı�Ե���䴦��
                {
                    if((h_left_lost == 0) || (h_right_lost == 0)) //ǰ������ȫ����
                    {
                        if(((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) < -10) && ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > -40) && (bian_line[H * 2 + 5] != 79)) //���ϰ�ͻ�䣨�ж���һ�εĵ��ǲ����ڱ�Ե��
                        {
                            tubian_num = bian_line[H * 2 + 5] - bian_line[H * 2 + 1];
                            tubian_num_H = H;
                            za_flag_r = 1;
                        }
                        if(((bian_line[H * 2] - bian_line[H * 2 + 4]) > 10) && ((bian_line[H * 2] - bian_line[H * 2 + 4]) < 40) && (bian_line[H * 2 + 4] != 0)) //���ϰ�ͻ�䣨�ж���һ�εĵ��ǲ����ڱ�Ե��
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
                if(((bian_line[H * 2] - bian_line[H * 2 + 4]) < -6) || ((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 6)) //�нϴ�ı�Ե���䴦��
                {
                    if((bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 6)
                    {
                        if((za_flag_r == 1) && (H > 15)) //�ж��Ƿ�Ϊ�ϰ�
                        {
                            if(tubian_num - (bian_line[H * 2 + 1] - bian_line[H * 2 + 5]) > 1) //�ж��ϴ�ͻ������εĲ���С
                            {
                                if(bian_line[H * 2 + 1] != 79) //�ж����䵽�ı߲��Ǳ�Ե
                                {
                                    if(tubian_num_H - H > 3) //ȷ��ͻ������
                                    {
                                        za_flag = 1; //ȷ���ҵ��ϰ�
                                    }
                                }
                            }
                        }
                        if(H < 45) //45�к���ͻ�䲹�ߴ���
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
                            if(tubian_num - (bian_line[H * 2 + 4] - bian_line[H * 2]) > 1) //�ж��ϴ�ͻ������εĲ���С
                            {
                                if(bian_line[H * 2] != 0) //�ж����䵽�ı߲��Ǳ�Ե
                                {
                                    if(tubian_num_H - H > 3) //ȷ��ͻ������
                                    {
                                        za_flag = 1; //ȷ���ҵ��ϰ�
                                    }
                                }
                            }
                        }
                        if(H < 45) //45�к���ͻ�䲹�ߴ���
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
                        center_line[H].x = center_line[H + 1].x; //������������
                }
                else  //�������ߴ���һ�߶��߻��߲����ߣ�
                {
                    if(((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] < 79)) || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))) //һ�߶���
                    {
                        if(bian_line[H * 2] < 1)              // ���һ�߶���
                        {
                            if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //���ߣ��������ж�
                            {
                                if(H < 45)
                                {
                                    shi_flag = 1;
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //������5�е�����
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x ;
                            }
                            else
                            {
                                if(H > 50)                    //ǰʮ���ж��Ƿ���Ҫ���м䲹
                                {
                                    if(h_left_lost == 1)        //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                        if(bian_line[H * 2 + 1] > 78)         // ��һ�߶���
                        {

                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    shi_flag = 1;
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //������5�е�����
                                    }
                                }
                                else
                                    center_line[H].x = center_line[H + 1].x ;
                            }
                            else
                            {
                                if(H > 50)                        //ǰʮ���ж��Ƿ���Ҫ���м䲹
                                {
                                    if(h_right_lost == 1)         //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79))            //���߾����ҵ�
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
                                    center_line[H + l].x = 0; //������5�е�����
                                }
                                center_line[H].x = center_line[H + 1].x;
                            }
                            else
                                center_line[H].x = center_line[H + 1].x;
                        }
                    }
                }
                if((left_lost == 1) && (right_lost == 1)) //���߶�ûɨ���ж��� ʮ��
                {
                    shi_flag = 1;

                    for(l = 0; l < 5; l++)
                    {
                        center_line[H + l].x = 0; //������5�е�����
                    }
                }
                point = center_line[H].x; //��̬ȡ��


                if(shi_flag == 1)
                {
                    if(H == 55)
                        point = 40;
                    else
                        point = center_line[H + 5].x; //ȡ�����ֵ��Ϊ��ʼɨ���
                }
            }
            else   //ʮ��ʱ  ������
            {
                point = center_line[H + 3].x;
                for(l = 0; l < 6; l++)
                {
                    center_line[H + l].x = 0; //������5�е�����
                }
            }
        }
        /*�����Ƿ�ʮ��ɨ��*/


        /*������ȷ����ʮ�ֺ�Ĵ���*/
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
            if((bian_line[H * 2] > 1) && (bian_line[H * 2 + 1] < 79)) //�ҵ�������
            {
                if((bian_line[H * 2 + 1] - bian_line[H * 2] - distence[H] * 2) < 10) //ȷ��ʮ���ϱߵı���
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
        /*������ȷ����ʮ�ֺ�Ĵ���*/

        ///////////////////////////////////////////////////////////////

        if(H == 1)
        {
            point = center_line[56].x;
        }


        /*����ȷ���ǻ�����Ľ�������*/
        if (Ring_Find_Flag == 1)
        {
            /*��������ߵ����*/
            if (LeftRing_Find_Flag == 1)
            {
                /*�����ǽ�������*/
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

                /*�ڻ��ڵ�Ѳ�߲��߲���*/
                if (assist3 == 1)
                {
                    assist2 = 0;
                    for(W = point; W > 0; W--) //����ɨ��
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

                    for (W = point; W < 80; W++) //����ɨ��
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

                        if(bian_line[H * 2 + 1] > 78) //�ұ߶���
                        {
                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //������5�е�����
                                    }
                                }

                                else
                                    center_line[H].x = center_line[H + 1].x + 5;
                            }

                            else
                            {
                                if(H > 50)                          //ǰʮ���ж��Ƿ���Ҫ���м䲹
                                {
                                    if(h_right_lost == 1)           //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //˫�߶�������
                        {
                            center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 5;
                        }

                        else
                        {
                            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //�������ļ�⵽�������ˣ�
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

            /*�������ұߵ����*/
            if (RightRing_Find_Flag == 1)
            {
                /*�����ǽ�������*/
                if (h_right_lost == 0 && h_left_lost == 0 && assistr0 == 1) //ǰ55�����߶�������
                {
                    assist_rADD = 1;
                    for(W = point; W > 0; W--) //����ɨ��
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

                if (assist_rADD == 1 && h_right_lost == 1 && h_left_lost == 0 )  //ǰ55���ұ߶��ߣ����û��
                {
                    assistr1 = 1;
                    assistr0 = 0;
                    for(W = point; W > 0; W--) //����ɨ��
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

                if (h_right_lost == 0 && assistr1 == 1)//ǰ55���ұ߶���
                {
                    assist_rADD = 0;
                    for (W = point; W < 80; W++) //����ɨ��
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

                if(assistr2 == 1 && h_right_lost == 0 && h_left_lost == 0) //���߶�û����
                {
                    assistr1 = 0;
                    assistr3 = 1;
                }

                /*�ڻ��ڵ�Ѳ�߲��߲���*/
                if (assistr3 == 1)
                {
                    assistr2 = 0;
                    for(W = point; W > 0; W--) //����ɨ��
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

                    for (W = point; W < 80; W++) //����ɨ��
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

                        if(bian_line[H * 2 + 1] > 78) //�ұ߶���
                        {
                            if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                            {
                                if(H < 45)
                                {
                                    for(l = 0; l < 5; l++)
                                    {
                                        center_line[H + l].x = 0; //������5�е�����
                                    }
                                }

                                else
                                    center_line[H].x = center_line[H + 1].x;
                            }

                            else
                            {
                                if(H > 50)                          //ǰʮ���ж��Ƿ���Ҫ���м䲹
                                {
                                    if(h_right_lost == 1)           //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                        if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //˫�߶�������
                        {
                            center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2;
                        }

                        else
                        {
                            if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78)) //�������ļ�⵽�������ˣ�
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

                            /*����Ϊ�󻷵���ȥ�Ĳ���*/
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
                                                       || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))//��һ�߶���
                                            ) 
                                            { 
//                                                        if(bian_line[H * 2] < 1) //��߶��ߣ��ұ߲�����
//                                                        { 
//                                                                       if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //���������������
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

                                                        if(bian_line[H * 2 + 1] > 78)//�ұ߶��ߣ���߲�����
                                                        { 
                                                                       if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                                                                       { 
                                                                                       center_line[H].x = center_line[H + 1].x + 10;
                                                                        } 
 
                                                                       else 
                                                                       { 
                                                                                    if(H > 50) //ǰʮ���ж��Ƿ���Ҫ���м䲹 
                                                                                    {
                                                                                                 if(h_right_lost == 1) //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                                           else //���߶����� ���� û����
                                           { 
                                                          
                                                         if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //û����
                                                         {
                                                                        center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 10;
                                                         }
                                                        
                                                         else //���߶�����
                                                         { 
                                                                        if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78))
                                                                        {
																		  
                                                                                     for(l = 0; l < 5; l++)
                                                                                     {
                                                                                                  center_line[H + l].x = center_line[H].x - 10; //������5�е�����
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
                            /*����Ϊ�󻷵���ȥ�Ĳ���*/ 

                            
                            /*����Ϊ�һ�����ȥ�Ĳ���*/ 
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
                                                       || ((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] > 78))//��һ�߶���
                                            ) 
                                            { 
                                                        if(bian_line[H * 2] < 1) //��߶��ߣ��ұ߲�����
                                                        { 
                                                                       if((bian_line[H * 2 + 1] - bian_line[H * 2 + 3]) > 1) //���������������
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

                                                        if(bian_line[H * 2 + 1] > 78)//�ұ߶��ߣ���߲�����
                                                        { 
                                                                       if((bian_line[H * 2] - bian_line[H * 2 + 2]) < -1)
                                                                       { 
                                                                                       center_line[H].x = center_line[H + 1].x + 10;
                                                                        } 
 
                                                                       else 
                                                                       { 
                                                                                    if(H > 50) //ǰʮ���ж��Ƿ���Ҫ���м䲹 
                                                                                    {
                                                                                                 if(h_right_lost == 1) //ǰ�����ж��ߣ����м䲹��û�������߲�
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
                                           else //���߶����� ���� û����
                                           { 
                                                          
                                                         if((bian_line[H * 2] > 0) && (bian_line[H * 2 + 1] < 79)) //û����
                                                         {
                                                                        center_line[H].x = (bian_line[H * 2] + bian_line[H * 2 + 1]) / 2 + 10;
                                                         }
                                                        
                                                         else //���߶�����
                                                         { 
                                                                        if((bian_line[H * 2] < 1) && (bian_line[H * 2 + 1] > 78))
                                                                        {
																		  
                                                                                     for(l = 0; l < 5; l++)
                                                                                     {
                                                                                                  center_line[H + l].x = 75; //������5�е�����
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


    /*�����ǻ������*/

        if (Ring_Find_Flag == 0)
        {
            for(Ring_Line_Value = 48; Ring_Line_Value > 28; Ring_Line_Value-- ) //48~29��
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

                }//�󻷵�ʶ��

                    zhuqi_2 = bian_line[Ring_Line_Value * 2 + 1] - bian_line[Ring_Line_Value * 2 + 7];
					zhuqi_3 = bian_line[Ring_Line_Value * 2 + 6] - bian_line[Ring_Line_Value * 2];
                if(
                    h_right_lost == 0 && h_left_lost == 0 //ǰ55�����߶�����
                    && right_lost == 1 && left_lost == 0//������ұ߶��ߣ���߲�����

                    && (bian_line[Ring_Line_Value * 2] > 1)    //��һ�������û����
                    && (bian_line[Ring_Line_Value * 2 + 2] > 1)//�ڶ��������û����
                    && (bian_line[Ring_Line_Value * 2 + 4] > 1)//�����������û����
                    && (bian_line[Ring_Line_Value * 2 + 6] > 1)//�����������û����
                    && (bian_line[Ring_Line_Value * 2 + 8] > 1)//�����������û����

					 && (bian_line[Ring_Line_Value * 2 + 1] > 78) //��һ���ұ��߶���
					 && (bian_line[Ring_Line_Value * 2 + 5] < 78) //�������ұ���û����



                    && ((bian_line[Ring_Line_Value * 2 + 1] - bian_line[Ring_Line_Value * 2 + 7]) > 5)//�ұߵ�һ��������в����20
                    && ((bian_line[Ring_Line_Value * 2 + 6] - bian_line[Ring_Line_Value * 2]) < 20)//��ߵ�һ��������в�С��3
					  

                )
                {
                    Ring_Find_Flag = 1;
                    RightRing_Find_Flag = 1;
                    assistr0 = 1;
                }//�һ���ʶ��
            }
        }






    /********��Ȩ��ƫ����*********/
    for(i = 0; i < 60; i ++)                                                          //������������߼�Ȩƽ��ֵ����
    {
        if((center_line[i].x > 1) && (center_line[i].x < 78)) //ֻ�����2~77��
        {
            sum_center += center_line[i].x;//��Ȩ
            num_ave ++;//�ӵĴ���
            if((i > (int)QZ_UP) && (i < (int)QZ_DOWN))    //������10~40�У��ǳ���Ч����
            {
                sum_center += center_line[i].x * 3; //һ�����ֵ���������
                num_ave += 3;
            }
        }
    }

    ave_center = sum_center / num_ave; //��ƽ��ֵ
    sum_center = 0; //����ֵ�� ��λ
    num_ave = 0; //�ӵĴ�����λ
    /*
    bianbi_num=0;
    R_max=40;
    L_min=40;

    for(i=0;i<60;i++)                                                            //������������߼�Ȩƽ��ֵ����
    {
    if((center_line[i].x>0)&&(center_line[i].x<79))                               //��������Ƿ���Ч��1~78��
    {
      if(center_line[i].x>R_max)     //�ҵ��ұ������ֵ
      {
        R_max=center_line[i].x;
      }
      if(center_line[i].x<L_min)     //�ҵ��������Сֵ
      {
        L_min=center_line[i].x;
      }

      if(num_ave==0)
      {
        speed_ave=i;                    //ɨ�赽��һ����Чֵ��ʱ��  ��¼����
       // bian_bi=(float)(60-speed_ave)/60.0;    //�����ɨ�赽����ռ�ı���
      }
      sum_center+=center_line[i].x;
      num_ave++;
      if((i>speed_ave)&&(i<speed_ave+(60-speed_ave)/2))     //ֻ��ǰһ���ߵ�ǰհ    //��Ȩ
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

     for(i=0;i<60;i++)                                                              //���߼�Ȩƽ��ֵ����
     {
       sit_L_F[i].x=sit_L[i].x;
       sit_R_F[i].x=sit_R[i].x;
       center_line_F[i].x=center_line[i].x;

       sit_L_F[i].y=sit_L[i].y+68;
       sit_R_F[i].y=sit_R[i].y+68;
       center_line_F[i].y=center_line[i].y+68;
     }
    for(i=0;i<60;i++)    //���������
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

    center = (uint16)dir_zhongzhi; //�����ֵ
    current_error = ave_center - 40; //�����ĵĲ�ֵ

    dir_kp=(uint16)(dir_kp1+(current_error*current_error)/20); //��̬Pֵ  		dir_kp1=1.6, dir_kp2=60,dir_kd=8;

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



    dir_out = (center + dir_kp * current_error + dir_kd * (current_error - last_error)); //PID�㷨

    if(dir_out < 0)dir_out = 0;

    dir = dir_out;

    if(dir > (center + 1033)) //�޷�
    {
        dir = (center + 1033);
    }
    if(dir < (center - 1033))
    {
        dir = (center - 1033);
    }

    last_error = current_error; //��¼�ϴεĲ�ֵ
    last_dir = dir; //��¼�ϴθ��Ķ��ֵ

    //dir_pianjiao=dir+100-(uint16)dir_zhongzhi;//���о�ûʲô��

     ftm_pwm_duty(FTM1, FTM_CH0,(uint16)dir);  // ʹ�ܶ�� 
   // ftm_pwm_duty(FTM1, FTM_CH0,5446);  // ���Զ��   4413  3380   5446


}

void img_extract(void *dst, void *src, uint32_t srclen)
{
    uint8_t colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    uint8_t *mdst = dst;
    uint8_t *msrc = src;
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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