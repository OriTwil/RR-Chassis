/*
 * @Author: szf
 * @Date: 2023-02-23 19:01:45
 * @LastEditTime: 2023-05-06 17:16:33
 * @LastEditors: szf
 * @Description: 全向轮底盘驱动
 * @FilePath: \RR-Chassis\Usercode\user_src\ow_chassis_control.c
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
#include <math.h>
#include "usercallback.h"
#include "usercalculate.h"
#include "ow_chassis_control.h"

/**
 * @description: 线程一：底盘控制
 * @author: szf
 * @date:
 * @return {void}
 */
void OwChassisControlTask(void const *argument)
{
    // 初始化
    PIDInit(); //PID参数初始化
    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);              // 以mavlink接收上位机通过串口发送的消息
    osDelay(20);

    // 解算，速度伺服
    for (;;) {

        //PID参数更新
        PIDUpdate();

        // 速度伺服
        speedServo(moter_speed[0], &hDJI[0]);
        speedServo(moter_speed[1], &hDJI[1]);
        speedServo(moter_speed[2], &hDJI[2]);
        speedServo(moter_speed[3], &hDJI[3]);

        CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output,
                             hDJI[1].speedPID.output,
                             hDJI[2].speedPID.output,
                             hDJI[3].speedPID.output);

        // A板上的绿灯作指示灯
        static int n_ = 0;
        if (n_++ > 100) {
            n_ = 0;
            HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
        }
        osDelay(1);
    }
}

void OwChassisTaskStart(mavlink_controller_t *ctrl_data)
{
    osThreadDef(chassiscontrol, OwChassisControlTask, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(chassiscontrol), ctrl_data);
}

void PIDInit()
{
    // 位置式pid参数设置
    pid_pos_w_pos.Kp    = -120;
    pid_pos_w_pos.Ki    = 0.0001;
    pid_pos_w_pos.Kd    = 0;
    pid_pos_w_pos.limit = 1;

    pid_pos_x_pos.Kp    = 4;
    pid_pos_x_pos.Ki    = 0.0001;
    pid_pos_x_pos.Kd    = 0;
    pid_pos_x_pos.limit = 1;

    pid_pos_y_pos.Kp    = 4;
    pid_pos_y_pos.Ki    = 0.0001;
    pid_pos_y_pos.Kd    = 0;
    pid_pos_y_pos.limit = 1;
}

void PIDUpdate()
{
    //位置式PID
    pid_pos_w_pos.target   = control.w_set;
    pid_pos_w_pos.feedback = mav_posture.zangle;
    pid_pos_x_pos.target   = control.x_set;
    pid_pos_x_pos.feedback = mav_posture.pos_x;
    pid_pos_y_pos.target   = control.y_set;
    pid_pos_y_pos.feedback = mav_posture.pos_y;
}