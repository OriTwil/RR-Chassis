/*
 * @Author: wtr电控组
 * @Date: 2022-10-13 23:33:37
 * @LastEditTime: 2023-05-13 14:55:49
 * @LastEditors: szf
 * @Description: 封装WTR曾经用过的解码函数
 * @FilePath: \RR-Chassis\Usercode\user_src\wtr_uart.c
 * @WeChat:szf13373959031
 */
#include "wtr_uart.h"
#include "wtr_dji.h"
#include "string.h"
#include "math.h"
#include "chassis_config.h"
#include "chassis_communicate.h"

Remote_t Raw_Data;
underpan_speed crl_speed;
uint8_t JoyStickReceiveData[18];

double angMax = 360;
double posRef; // 工程中也可调用需要参数来传值
float vx, vy, vw;
Posture posture;
uint8_t ch[1];
static uint8_t count = 0;
uint8_t i            = 0;

// AS69
void AS69_Decode()
{

    Raw_Data.ch0 = ((int16_t)JoyStickReceiveData[0] | ((int16_t)JoyStickReceiveData[1] << 8)) & 0x07FF;
    Raw_Data.ch1 = (((int16_t)JoyStickReceiveData[1] >> 3) | ((int16_t)JoyStickReceiveData[2] << 5)) & 0x07FF;
    Raw_Data.ch2 = (((int16_t)JoyStickReceiveData[2] >> 6) | ((int16_t)JoyStickReceiveData[3] << 2) |
                    ((int16_t)JoyStickReceiveData[4] << 10)) &
                   0x07FF;
    Raw_Data.ch3   = (((int16_t)JoyStickReceiveData[4] >> 1) | ((int16_t)JoyStickReceiveData[5] << 7)) & 0x07FF;
    Raw_Data.left  = ((JoyStickReceiveData[5] >> 4) & 0x000C) >> 2;
    Raw_Data.right = ((JoyStickReceiveData[5] >> 4) & 0x0003);
    Raw_Data.wheel = ((int16_t)JoyStickReceiveData[16]) | ((int16_t)JoyStickReceiveData[17] << 8);

    /* UART1 callback decode function  */
}

void DJI_Control()
{
    switch (Raw_Data.left) {
        case 1:
            // speed = (float ) (Raw_Data.ch0 - CH0_BIAS)/CH_RANGE * 200;
            crl_speed.vx = (float)((Raw_Data.ch0 - CH0_BIAS) / CH_RANGE * 1);
            crl_speed.vy = (float)(Raw_Data.ch1 - CH1_BIAS) / CH_RANGE * 1;
            /* left choice 1 */
            break;
        case 3:
            posRef = (double)(Raw_Data.wheel - CH1_BIAS) / CH_RANGE * angMax;
            /* left choice 2 */
            break;
        case 2:
            posRef = (double)(Raw_Data.wheel - CH2_BIAS) / CH_RANGE * angMax;
            /* left choice 3 */
            break;
        default:
            break;
    }

    switch (Raw_Data.right) {
        case 1:
            crl_speed.vw = (float)(Raw_Data.ch2 - CH2_BIAS) / CH_RANGE * 1;
            /* right choice 1 */
            break;
        case 3:
            angMax = 180;
            /* right choice 2 */
            break;
        case 2:
            angMax = 90;
            /* right choice 3 */
            break;
        default:
            break;
    }
}
// OPS全方位平面定位系统
void OPS_Decode()
{
    HAL_UART_Receive_IT(&huart_OPS, (uint8_t *)&ch, 1);
    // USART_ClearITPendingBit( USART1, USART_FLAG_RXNE);
    // HAL_UART_IRQHandler(&huart6); // 该函数会清空中断标志，取消中断使能，并间接调用回调函数
    switch (count) // uint8_t隐转为int
    {
        case 0:

            if (ch[0] == 0x0d)
                count++;
            else
                count = 0;
            break;

        case 1:

            if (ch[0] == 0x0a) {
                i = 0;
                count++;
            } else if (ch[0] == 0x0d)
                ;
            else
                count = 0;
            break;

        case 2:

            posture.data[i] = ch[0];
            i++;
            if (i >= 24) {
                i = 0;
                count++;
            }
            break;

        case 3:

            if (ch[0] == 0x0a)
                count++;
            else
                count = 0;
            break;

        case 4:

            if (ch[0] == 0x0d) {
                mav_posture.zangle = posture.ActVal[0];
                // mav_posture.xangle = posture.ActVal[1] * 0.001;
                mav_posture.xangle = control.x_set;
                // mav_posture.yangle = posture.ActVal[2] * 0.001;
                mav_posture.yangle = control.y_set;
                mav_posture.pos_x  = posture.ActVal[3] * 0.001;
                mav_posture.pos_y  = posture.ActVal[4] * 0.001;
                mav_posture.w_z    = posture.ActVal[5] * 0.001;

                chassis_data.zangle = posture.ActVal[0] * 0.001;
                chassis_data.xangle = posture.ActVal[1] * 0.001;
                chassis_data.yangle = posture.ActVal[2] * 0.001;
                chassis_data.pos_x  = posture.ActVal[3] * 0.001;
                chassis_data.pos_y  = posture.ActVal[4] * 0.001;
                chassis_data.w_z    = posture.ActVal[5] * 0.001;
            }
            count = 0;
            break;

        default:

            count = 0;
            break;
    }
}

void Struct(uint8_t strDestination[], char strSource[], int num)
{
    int i = 0, j = 0;

    while (strDestination[i] != '\0') {
        i++;
    }

    for (j = 0; j < num; j++) {
        strDestination[i++] = strSource[j];
    }
}

void Update_X(float New_X)
{
    uint8_t Update_X[8] = "ACTX";

    static union
    {
        /* data */
        float X;
        char data[4];
    }New_set;
    
    New_set.X = New_X;

    Struct(Update_X,New_set.data,4);
    HAL_UART_Transmit(&huart_OPS,Update_X,8,50);
}

void Update_Y(float New_Y)
{
    uint8_t Update_Y[8] = "ACTY";

    static union
    {
        /* data */
        float Y;
        char data[4];
    }New_set;
    
    New_set.Y = New_Y;

    Struct(Update_Y,New_set.data,4);
    HAL_UART_Transmit(&huart_OPS,Update_Y,8,50);
}

void Update_A(float New_A)
{
    uint8_t Update_A[8] = "ACTJ";

    static union
    {
        /* data */
        float A;
        char data[4];
    }New_set;
    
    New_set.A = New_A;

    Struct(Update_A,New_set.data,4);
    HAL_UART_Transmit(&huart_OPS,Update_A,8,50);
}