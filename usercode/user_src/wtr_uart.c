#include "wtr_uart.h"
#include "DJI.h"
#include "string.h"
#include "math.h"
#include "usart.h"

Remote_t Raw_Data;
underpan_speed crl_speed;
uint8_t JoyStickReceiveData[18];

double angMax = 360;
double posRef ;//工程中也可调用需要参数来传值
float vx,vy,vw;
void UART1Decode(){

    Raw_Data.ch0 = ((int16_t)JoyStickReceiveData[0] | ((int16_t)JoyStickReceiveData[1] << 8)) & 0x07FF; 
    Raw_Data.ch1 = (((int16_t)JoyStickReceiveData[1] >> 3) | ((int16_t)JoyStickReceiveData[2] << 5)) & 0x07FF;
    Raw_Data.ch2 = (((int16_t)JoyStickReceiveData[2] >> 6) | ((int16_t)JoyStickReceiveData[3] << 2) |
                                                ((int16_t)JoyStickReceiveData[4] << 10)) & 0x07FF;
    Raw_Data.ch3 = (((int16_t)JoyStickReceiveData[4] >> 1) | ((int16_t)JoyStickReceiveData[5]<<7)) & 0x07FF;
    Raw_Data.left = ((JoyStickReceiveData[5] >> 4) & 0x000C) >> 2;
    Raw_Data.right = ((JoyStickReceiveData[5] >> 4) & 0x0003);
    Raw_Data.wheel = ((int16_t)JoyStickReceiveData[16]) | ((int16_t)JoyStickReceiveData[17] << 8);

    switch(Raw_Data.left)
    {
        case 1:
            //speed = (float ) (Raw_Data.ch0 - CH0_BIAS)/CH_RANGE * 200;
            crl_speed.vx = (float ) ((Raw_Data.ch0 - CH0_BIAS)/CH_RANGE * 1500);
            crl_speed.vy = (float ) (Raw_Data.ch1 - CH1_BIAS)/CH_RANGE * 1500;
            /* left choice 1 */
            break;
        case 3:
            posRef = (double ) (Raw_Data.wheel - CH1_BIAS)/CH_RANGE * angMax;
            /* left choice 2 */
            break;
        case 2:
            posRef = (double ) (Raw_Data.wheel - CH2_BIAS)/CH_RANGE * angMax;
            /* left choice 3 */
            break;
        default:
            break;
    }

    switch(Raw_Data.right)
    {
        case 1:
            crl_speed.vw = (float ) (Raw_Data.ch2 - CH2_BIAS)/CH_RANGE * 5000;
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

    /* UART1 callback decode function  */
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
void mavlink_msg_decode(msg, &StructReceived)
{
    crl_speed.vx = 0;
    crl_speed.vy = 0;
    crl_speed.vw = 0;
}

//    
//}


