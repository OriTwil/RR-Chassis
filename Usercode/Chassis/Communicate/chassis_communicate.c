/**
 * @file chassis_communicate.c
 * @author OriTwil
 * @brief 通信组件
 * @version 1.0
 * @date 2023-5-18
 *
 */

#include "chassis_communicate.h"
#include "chassis_commen.h"
#include "chassis_config.h"
#include "wtr_uart.h"
#include "chassis_remote_control.h"

// 变量定义
mavlink_control_t control;               // 上位机规划后的控制信息
mavlink_posture_t mav_posture;           // 定位系统的信息
mavlink_chassis_to_upper_t chassis_data; // 板间通信

// 拷贝变量
mavlink_posture_t posture_temp;
mavlink_chassis_to_upper_t chassis_data_temp;
mavlink_joystick_air_t msg_joystick_air_temp;
void CommunicateTask(void const *argument)
{
    vTaskDelay(20);
    for (;;) {
        vPortEnterCritical();
        posture_temp          = mav_posture;  // 定位数据拷贝
        chassis_data_temp     = chassis_data; // 板间通信数据拷贝
        msg_joystick_air_temp = Msg_joystick_air.msg_joystick_air; // 遥控器数据拷贝
        vPortExitCritical();

        mavlink_msg_joystick_air_send_struct(MAVLINK_COMM_2, &Msg_joystick_air.msg_joystick_air); // 板间通信
        vTaskDelay(10);
        // mavlink_msg_chassis_to_upper_send_struct(MAVLINK_COMM_2, &chassis_data_temp); // 板间通信
        mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &posture_temp);               // 定位信息发到上位机
        vTaskDelay(10); // 通信线程的频率不能太高
    }
}

void CommunicateTaskStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityBelowNormal, 0, 1024);
    osThreadCreate(osThread(communicate), NULL);
}

void CommunicateInit()
{
    control.vw_set = 0;
    control.vx_set = 0;
    control.vy_set = 0;
    control.w_set  = 0;
    control.x_set  = 0;
    control.y_set  = 0;

    mav_posture.point  = 0;
    mav_posture.pos_x  = 0.0;
    mav_posture.pos_y  = 0.0;
    mav_posture.zangle = 0.0;

    wtrMavlink_BindChannel(&huart_Computer, MAVLINK_COMM_0);         // 上位机MAVLINK初始化
    wtrMavlink_BindChannel(&huart_Chassis_to_Upper, MAVLINK_COMM_2); // 板间通信MAVLINK初始化

    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0); // 以MAVLINK接收上位机通过串口发送的消息
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_2); // 板间通信MAVLINK

    HAL_UART_Receive_DMA(&huart_AS69, JoyStickReceiveData, 18); // DMA接收AS69 DJI
}
