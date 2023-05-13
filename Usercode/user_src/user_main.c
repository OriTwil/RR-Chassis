/**
 * @Author: szf
 * @Date: 2023-02-22 10:59:01
 * @LastEditTime: 2023-02-22 19:27:40
 * @LastEditors: szf
 * @Description: E
 * @FilePath: \ER\Usercode\user_src\usermain.c
 * @WeChat:szf13373959031
 **/

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "user_main.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "user_callback.h"
#include "chassis_control.h"
#include "chassis_driver.h"
#include "chassis_perception.h"
#include "CLI.h"

// 变量定义
mavlink_control_set_t control;
mavlink_speed_control_status_t v_state;
mavlink_posture_t mav_posture;
mavlink_channel_t CtrlDataSendChan = MAVLINK_COMM_0;

uint8_t i = 0;

PID_Pos pid_pos_w_pos;
PID_Pos pid_pos_x_pos;
PID_Pos pid_pos_y_pos;

mavlink_controller_t ControllerData = {0};

/**
 * @description: 初始化与开启线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{

    CANFilterInit(&hcan1);
    hDJI[0].motorType = M3508;
    hDJI[1].motorType = M3508;
    hDJI[2].motorType = M3508;
    hDJI[3].motorType = M3508;
    hDJI[4].motorType = M3508;
	hDJI[5].motorType = M3508;
    hDJI[6].motorType = M3508;
    DJI_Init();// 大疆电机初始化

    wtrMavlink_BindChannel(&huart8, MAVLINK_COMM_0);// MAVLINK初始化
    CtrlDataSender_Init(&huart2, MAVLINK_COMM_1); // 遥控器初始化
    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69

    //开启线程
    OwChassisTaskStart(&ControllerData);// 全向轮底盘控制线程
    // PerceptionTaskStart(&ControllerData);  // 底盘感知定位线程
    // ChassisTaskStart(&ControllerData); //舵轮底盘控制线程
	// CommunicateTaskStart(&ControllerData);// 遥控器线程
    // StateManagemanttaskStart(&ControllerData);

    for (;;) {
        osDelay(1);
    }
}

void CtrlDataSender_Init(UART_HandleTypeDef *huart, mavlink_channel_t chan)
{
	// WTR_MAVLink_Init(huart, chan);
	wtrMavlink_BindChannel(huart, MAVLINK_COMM_1);// MAVLINK初始化
	CtrlDataSendChan = chan;
}

void BlinkLED()
{
    static int n_ = 0;
    if (n_++ > 100) {
        n_ = 0;
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
    }
}