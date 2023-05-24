/**
 * @file chassis_communicate.c
 * @author OriTwil
 * @brief 通信组件
 * @version 1.0
 * @date 2023-5-18
 *
 */

#include "chassis_communicate.h"
#include "user_main.h"
#include "chassis_state_management.h"

// 变量定义
mavlink_control_t control;                             // 上位机规划后的控制信息
mavlink_posture_t mav_posture;                         // 定位系统的信息
mavlink_chassis_to_upper_t chassis_data;               // todo 遥控器
mavlink_channel_t CtrlDataSendChan   = MAVLINK_COMM_0; // todo MAVLINK分配与测试
mavlink_channel_t ChassisToUpperChan = MAVLINK_COMM_1;
mavlink_posture_t posture_temp;

void CommunicateTask(void const *argument)
{
    vTaskDelay(20);
    for (;;) {
        vPortEnterCritical();
        if (Raw_Data.left == 1 /* 判断按键1是否按下 */) {
            mav_posture.point = 1;
        }
        if (Raw_Data.left == 2 /* 判断按键2是否按下 */) {
            mav_posture.point = 2;
        }
        if (Raw_Data.left == 3 /* 判断按键3是否按下 */) {
            mav_posture.point = 3;
        }
        posture_temp = mav_posture; // 定位数据拷贝
        vPortExitCritical();

        // mavlink_msg_controller_send_struct(CtrlDataSendChan, argument);
        // mavlink_msg_chassis_to_upper_send_struct(MAVLINK_COMM_2, &chassis_data); // 板间通信
        mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &posture_temp);          // 定位信息发到上位机
        vTaskDelay(10);
    }
}

void CommunicateTaskStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
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
    mav_posture.pos_x  = 0;
    mav_posture.pos_y  = 0;
    mav_posture.zangle = 0;

    // WTR_MAVLink_Init(huart, chan);
    wtrMavlink_BindChannel(&huart_Remote_Control, MAVLINK_COMM_1);   // 遥控器MAVLINK初始化
    wtrMavlink_BindChannel(&huart_Computer, MAVLINK_COMM_0);         // 上位机MAVLINK初始化
    wtrMavlink_BindChannel(&huart_Chassis_to_Upper, MAVLINK_COMM_2); // 板间通信MAVLINK初始化

    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0); // 以mavlink接收上位机通过串口发送的消息
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_1); // 接收遥控器MAVLINK
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_2); // 板间通信MAVLINK

    HAL_UART_Receive_DMA(&huart_AS69, JoyStickReceiveData, 18); // DMA接收AS69
}
