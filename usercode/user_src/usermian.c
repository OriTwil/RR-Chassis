/*
 * @Author: szf
 * @Date: 2022-10-22 13:54:44
 * @LastEditTime: 2022-12-12 22:47:16
 * @LastEditors: szf
 * @Description:
 * @FilePath: \underpan_v3.1\usercode\user_src\usermian.c
 * @WeChat:szf13373959031
 */
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
#include "ADS1256.h"
#define pi         3.1415926535898
#define DEC        (pi / 180)
#define r_underpan 0.1934
#define r_wheel    0.076
int test = 0;
int counter = 0;
float w_speed = 0;
/**
 * @description:
 * @author: szf
 * @date:
 * @return {*}
 */
void calculate_3(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w)
{
    moter_speed[0] = (-v_x * sin(30 * DEC) - v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * pi * r_wheel);
    moter_speed[1] = (+v_x + v_w * r_underpan_3) / (2 * pi * r_wheel);
    moter_speed[2] = (-v_x * sin(30 * DEC) + v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * pi * r_wheel);
}

/**
 * @description:
 * @author: szf
 * @date:
 * @return {void}
 */
void calculate_3_2(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w)
{
    moter_speed[2] = (-v_y * sin(30 * DEC) + v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
    moter_speed[1] = (+v_y + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
    moter_speed[0] = (-v_y * sin(30 * DEC) - v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
}

/**
 * @description:
 * @author: szf
 * @date:
 * @return {void}
 */
void calculate_4(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w)
{
    moter_speed[0] = (v_y + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[1] = (-v_x + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[2] = (-v_y + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[3] = (v_x + v_w * r_underpan_4) / (2 * pi * r_wheel);
}

/**
 * @description:
 * @author: szf
 * @date:
 * @return {void}
 */
void calculate_4_2(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w)
{
    moter_speed[0] = (vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[1] = (-vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[2] = (-vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[3] = (vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
}

/**
 * @description: 线程一：底盘控制
 * @author: szf
 * @date: 
 * @return {void}
 */
void thread_1(void const *argument)
{
    // 初始化设置
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;
    hDJI[3].motorType = M3508;
    DJI_Init();
    wtrMavlink_BindChannel(&huart8, MAVLINK_COMM_0);
    // 串口接收信息

    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);//以mavlink接收
    osDelay(100);

    
    //编码器AMT102


    // 解算，速度伺服
    for (;;) {

        calculate_4_2(moter_speed, crl_speed.vx, crl_speed.vy, crl_speed.vw);
        // calculate_3_2(moter_speed,v_set.vx_set,v_set.vy_set,v_set.vw_set);//mavlink

        speedServo(moter_speed[0], &hDJI[0]);
        speedServo(moter_speed[1], &hDJI[1]);
        speedServo(moter_speed[2], &hDJI[2]);
        speedServo(moter_speed[3], &hDJI[3]);

        CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output,
                             hDJI[1].speedPID.output,
                             hDJI[2].speedPID.output,
                             hDJI[3].speedPID.output);
        // 电机速度反馈，可以正向解算，传底盘的速度
        //  mavlink_speed_control_status_t speed_t;
        static int cnt = 0;
        if (cnt++ > 50) {
            cnt = 0;
            // speed_t.vx_state = hDJI[0].FdbData.rpm;
            // speed_t.vy_state = hDJI[1].FdbData.rpm;
            // speed_t.vw_state = hDJI[2].FdbData.rpm;

            // char ch[] = "123456\n";
            // HAL_UART_Transmit(&huart8, (uint8_t*)&ch,7,100);

            // mavlink_msg_speed_control_status_send_struct(MAVLINK_COMM_0, &speed_t);
            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
        }
        osDelay(1);
    }
}

/**
 * @description: 线程二：定位系统
 * @author: szf
 * @date: 
 * @return {void}
 */
void thread_2(void const *argument)
{
    // 码盘定位系统通过串口6收发信息
    HAL_UART_Receive_IT(&huart6, (uint8_t *)&ch, 1);
    
    // DT35距离传感器
    // 初始化
    ADS1256_Init();
    ADS1256_UpdateDiffData();

    for (;;) {
        osDelay(100);
    }
}

/**
 * @description: 创建线程
 * @author: szf
 * @date: 
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    osThreadDef(speedservo, thread_1, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(speedservo), NULL);

    osThreadDef(position, thread_2, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(position), NULL);

    for (;;) {
        osDelay(1);
    }
}

/**
 * @description: 串口回调函数，解码
 * @param {UART_HandleTypeDef} *huart
 * @author: szf
 * @date:
 * @return {void}
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    test++;

    // 上位机消息
    if (huart->Instance == UART8) {
        // UART1Decode();//AS69解码
        wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_0); // 进入mavlink回调
    }
    // 定位模块消息
    else if (huart->Instance == USART6) // 底盘定位系统的decode,可以换为DMA轮询,封装到祖传的串口库里s
    {
        // HAL_UART_Receive_IT(&huart6,(uint8_t *)&ch,1);
        // USART_ClearITPendingBit( USART1, USART_FLAG_RXNE);
        HAL_UART_IRQHandler(&huart6); // 该函数会清空中断标志，取消中断使能，并间接调用回调函数
        switch (count)                // uint8_t隐转为int
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
                    mav_posture.xangle = posture.ActVal[1];
                    mav_posture.yangle = posture.ActVal[2];
                    mav_posture.pos_x = posture.ActVal[3];
                    mav_posture.pos_y = posture.ActVal[4];
                    mav_posture.w_z = posture.ActVal[5];
                    mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &mav_posture); // 可能要调整延时
                }
                count = 0;
                break;

            default:

                count = 0;
                break;
        }
    } else {
        void AS69_Decode(); // AS69解码
    }
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    counter++;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2) {
    w_speed = counter / 2048 / 0.02;
    counter = 0;
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}