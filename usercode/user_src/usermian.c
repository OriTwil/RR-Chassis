#include "main.h"
#include "usermian.h"
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
#define pi 3.1415926535898
#define DEC (pi/180)
#define r_underpan 0.1934
#define r_wheel 0.076

double moter_speed [3];


void calculate(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[0] = (- v_x * sin(30 * DEC) - v_y * cos(30 * DEC)  + v_w * r_underpan)/(2 * pi * r_wheel);
    moter_speed[1] = (+ v_x                        + v_w * r_underpan)/(2 * pi * r_wheel);
    moter_speed[2] = (- v_x * sin(30 * DEC) + v_y * cos(30 * DEC)  + v_w * r_underpan)/(2 * pi * r_wheel);
}

void thread_1(void const * argument)
{
    CANFilterInit(&hcan1);//è¿‡æ»¤å™¨è®¾ï¿???
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;//ç”µæœºç±»åž‹è®¾ç½®
    DJI_Init();
    
    HAL_UART_Receive_DMA(&huart1,JoyStickReceiveData,18);
	osDelay(100);
    
    for(;;){
    calculate(moter_speed,crl_speed.vx,crl_speed.vy,crl_speed.vw);
    speedServo(moter_speed[0],&hDJI[0]);
    speedServo(moter_speed[1],&hDJI[1]);
    speedServo(moter_speed[2],&hDJI[2]);

    CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,
                                hDJI[1].speedPID.output,
                                hDJI[2].speedPID.output,
                                0);
    osDelay(1);
    }

    

}


void StartDefaultTask(void const * argument)
{
    osThreadDef(underpan, thread_1, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(underpan), NULL);


    for(;;)
    {
        osDelay(1);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART1Decode();
}