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
mavlink_control_t control;               // 上位机规划后的控制信息
mavlink_posture_t mav_posture;           // 定位系统的信息
mavlink_chassis_to_upper_t chassis_data; // todo 遥控器
mavlink_channel_t CtrlDataSendChan   = MAVLINK_COMM_0; //todo MAVLINK分配与测试
mavlink_channel_t ChassisToUpperChan = MAVLINK_COMM_1;

void CommunicateTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    HAL_UART_Receive_DMA(&huart_AS69, JoyStickReceiveData, 18); // DMA接收AS69
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_0);                  // 以mavlink接收上位机通过串口发送的消息
    while (1) {
        vPortEnterCritical();
        // mavlink_msg_controller_send_struct(CtrlDataSendChan, argument);
        // mavlink_msg_chassis_to_upper_send_struct(MAVLINK_COMM_1,chassis_data);
        mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &mav_posture); // 位置信息发到上位机
        vPortExitCritical();
        vTaskDelayUntil(&PreviousWakeTime, 4);
    }
}

void CommunicateTaskStart()
{
    osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(communicate), NULL);
}
