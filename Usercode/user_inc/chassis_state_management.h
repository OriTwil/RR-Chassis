/*** 
 * @Author: szf
 * @Date: 2023-05-13 15:33:55
 * @LastEditTime: 2023-05-13 21:49:30
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \RR-Chassis\Usercode\user_inc\chassis_state_management.h
 * @@WeChat:szf13373959031
 */
#ifndef UPPER_STATE_MANAGEMENT
#define UPPER_STATE_MANAGEMENT

#include "user_main.h"
#include "chassis_control_ow.h"
#include "chassis_perception.h"
#include "semphr.h"

typedef enum{
    Receive,
    Transmit
}PERCEPTION_STATE;

typedef enum
{
    Locked,
    RemoteControl,
    ComputerControl
}CHASSIS_STATE;

typedef struct{
    PERCEPTION_STATE Perception_state;
    CHASSIS_STATE Chassis_state;
    SemaphoreHandle_t  xMutex_Robot;
}ROBOT_STATE;

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state,ROBOT_STATE *current_robot_state);

void ChassisSwitchState(CHASSIS_STATE target_chassis_state,ROBOT_STATE *current_robot_state);

extern ROBOT_STATE Robot_state;
#endif