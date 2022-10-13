#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "usermain.h"
#define pi 3.1415926535898
#define DEC (pi/180)
#define r_underpan 0.1934
#define r_wheel 0.076

// //
// double moter_speed [4];

//将底盘速度解算到电机速度
void calculate_3(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[0] = (- v_x * sin(30 * DEC) - v_y * cos(30 * DEC)  + v_w * r_underpan_3)/(2 * pi * r_wheel);
    moter_speed[1] = (+ v_x                                        + v_w * r_underpan_3)/(2 * pi * r_wheel);
    moter_speed[2] = (- v_x * sin(30 * DEC) + v_y * cos(30 * DEC)  + v_w * r_underpan_3)/(2 * pi * r_wheel);
}//三轮全向

void calculate_4(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[0] = ( v_y + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[1] = (-v_x + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[2] = (-v_y + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[3] = ( v_x + v_w * r_underpan_4)/(2 * pi * r_wheel);
}//四轮全向

//线程一：底盘控制
void thread_1(void const * argument)
{
    //初始化设置
    CANFilterInit(&hcan1);//è¿‡æ»¤å™¨è®¾ï¿???
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;//ç”µæœºç±»åž‹è®¾ç½®
    hDJI[3].motorType = M3508;
    DJI_Init();
    //串口接收信息

    HAL_UART_Receive_DMA(&huart1,JoyStickReceiveData,18);
	osDelay(100);
    //解算，速度伺服
    for(;;){

    calculate_4(moter_speed,crl_speed.vx,crl_speed.vy,crl_speed.vw);
    speedServo(moter_speed[0],&hDJI[0]);
    speedServo(moter_speed[1],&hDJI[1]);
    speedServo(moter_speed[2],&hDJI[2]);
    speedServo(moter_speed[3],&hDJI[3]);

    CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,
                                hDJI[1].speedPID.output,
                                hDJI[2].speedPID.output,
                                hDJI[3].speedPID.output);

    // CanTransmit_DJI_5678(&hcan1,hDJI[0].speedPID.output,
    //                             hDJI[1].speedPID.output,
    //                             hDJI[2].speedPID.output,
    //                             hDJI[3].speedPID.output);  

    osDelay(1);
    }

    

}

//创建线程
void StartDefaultTask(void const * argument)
{
    osThreadDef(underpan, thread_1, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(underpan), NULL);


    for(;;)
    {
        osDelay(1);
    }
}
//串口回调函数，解码
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART1Decode();
}