/*
 * @Author: szf
 * @Date: 2023-02-23 19:13:13
 * @LastEditTime: 2023-05-13 19:54:06
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \RR-Chassis\Usercode\user_src\chassis_perception.c
 * @WeChat:szf13373959031
 */
#include "chassis_perception.h"
#include "chassis_commen.h"
#include "chassis_state_machine.h"
#include "chassis_config.h"

uint32_t test_pos[6] = {0};
ROBOT_STATE Robot_state_temp;

/**
 * @description: 定位系统
 * @author: szf
 * @date:
 * @return {void}
 */
void ChassisPerceptionTask(void const *argument)
{
    vTaskDelay(100);
    for (;;) {
        Robot_state_temp = ReadRobotState(&Robot_state);
        switch(Robot_state_temp.Perception_state)
        {
            case Receive:
            break;
            case Transmit:
            // 更新码盘
            break;
        }
        vTaskDelay(10);
    }
}

void PerceptionTaskStart()
{
    osThreadDef(perception, ChassisPerceptionTask, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(perception), NULL);
}

void PerceptionInit()
{
    // 码盘定位系统通过串口收信息
    HAL_UART_Receive_IT(&huart_OPS, (uint8_t *)&ch, 1);
}

/**
 * @description: 切换传感器状态
 * @param target_perception_state 目标状态(Receive Transmit)
 * @return {void}
 */
void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake(current_robot_state->xMutex_Robot, (TickType_t)10);
    current_robot_state->Perception_state = target_perception_state;
    xSemaphoreGive(current_robot_state->xMutex_Robot);
}