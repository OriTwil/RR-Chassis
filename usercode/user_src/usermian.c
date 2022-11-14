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
#include "wtr_mavlink.h"

//将三轮底盘速度解算到电机速度
void calculate_3(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[0] = (- v_x * sin(30 * DEC) - v_y * cos(30 * DEC)  + v_w * r_underpan_3)/(2 * pi * r_wheel);
    moter_speed[1] = (+ v_x                                        + v_w * r_underpan_3)/(2 * pi * r_wheel);
    moter_speed[2] = (- v_x * sin(30 * DEC) + v_y * cos(30 * DEC)  + v_w * r_underpan_3)/(2 * pi * r_wheel);
}
//三轮解算 另一种坐标方向
void calculate_3_2(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[2] = (- v_y * sin(30 * DEC) + v_x * cos(30 * DEC)  + v_w * r_underpan_3)*60/(2 * pi * r_wheel)*19;
    moter_speed[1] = (+ v_y                                        + v_w * r_underpan_3)*60/(2 * pi * r_wheel)*19;
    moter_speed[0] = (- v_y * sin(30 * DEC) - v_x * cos(30 * DEC)  + v_w * r_underpan_3)*60/(2 * pi * r_wheel)*19;
}


//将四轮底盘速度解算到电机速度
void calculate_4(double * moter_speed,
               double v_x,
               double v_y,
               double v_w)
{
    moter_speed[0] = ( v_y + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[1] = (-v_x + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[2] = (-v_y + v_w * r_underpan_4)/(2 * pi * r_wheel);
    moter_speed[3] = ( v_x + v_w * r_underpan_4)/(2 * pi * r_wheel);
}

//线程一：底盘控制
void thread_1(void const * argument)
{
    //初始化设置
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;
    hDJI[3].motorType = M3508;
    DJI_Init();
    wtrMavlink_BindChannel(&huart8, MAVLINK_COMM_0);
    //串口接收信息

    // HAL_UART_Receive_DMA(&huart1,JoyStickReceiveData,18);//DMA接收AS69
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);//以mavlink接收
	osDelay(100);

    //解算，速度伺服
    for(;;){

    calculate_3_2(moter_speed,v_set.vx_set,v_set.vy_set,v_set.vw_set);//mavlink

    speedServo(moter_speed[0],&hDJI[0]);
    speedServo(moter_speed[1],&hDJI[1]);
    speedServo(moter_speed[2],&hDJI[2]);
    speedServo(moter_speed[3],&hDJI[3]);

    CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,
                                hDJI[1].speedPID.output,
                                hDJI[2].speedPID.output,
                                hDJI[3].speedPID.output);

    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
    }
    

    // CanTransmit_DJI_5678(&hcan1,hDJI[0].speedPID.output,
    //                             hDJI[1].speedPID.output,
    //                             hDJI[2].speedPID.output,
    //                             hDJI[3].speedPID.output);  

    osDelay(1);
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
    // UART1Decode();//AS69解码
    wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_0);//进入mavlink回调
}

/**
 * @brief 接收到完整消息且校验通过后会调用这个函数。在这个函数里调用解码函数就可以向结构体写入收到的数据
 *
 * @param msg 接收到的消息
 * @return
 */

void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg)
{
    switch (msg->msgid) {
        case 9:
            // id = 9 的消息对应的解码函数(mavlink_msg_xxx_decode)
            mavlink_msg_speed_control_set_decode(msg, &v_set);
            break;
        case 2:
            // id = 2 的消息对应的解码函数(mavlink_msg_xxx_decode)
            break;
        // ......
        default:
            break;
    }
}

