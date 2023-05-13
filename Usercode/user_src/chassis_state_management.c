/*
 * @Author: szf
 * @Date: 2023-05-11 16:08:17
 * @LastEditTime: 2023-05-11 16:19:38
 * @LastEditors: szf
 * @Description: 任务切换线程
 * @FilePath: \RR-Upper-Structure-A\UserCode\user_src\chassis_state_management.c
 * @WeChat:szf13373959031
 */
#include "chassis_state_management.h"

ROBOT_STATE Robot_state = {
    .Perception_state = Receive,
    .Chassis_state = Locked,
    .xMutex_Robot = NULL
};

void StateManagemantTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    osDelay(20);
    for(;;)
    {
        osDelayUntil(&PreviousWakeTime, 5);
    }
}

void StateManagemantTestTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    osDelay(20);
    for(;;)
    {
        osDelayUntil(&PreviousWakeTime, 5);
    }
}

void StateManagemanttaskStart(mavlink_controller_t *controldata)
{
    Robot_state.xMutex_Robot = xSemaphoreCreateMutex();

    osThreadDef(statemanagement, StateManagemantTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(statemanagement), NULL);

    // osThreadDef(statemanagementtest,StateManagemantTestTask,osPriorityBelowNormal,0,512);
    // osThreadCreate(osThread(statemanagementtest),NULL);
}

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state,ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake( current_robot_state->xMutex_Robot, ( TickType_t ) 10 );
    current_robot_state->Perception_state = target_perception_state;
    xSemaphoreGive( current_robot_state->xMutex_Robot );
}

void ChassisSwitchState(CHASSIS_STATE target_chassis_state,ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake( current_robot_state->xMutex_Robot, ( TickType_t ) 10 );
    current_robot_state->Chassis_state = target_chassis_state;
    xSemaphoreGive( current_robot_state->xMutex_Robot );
}