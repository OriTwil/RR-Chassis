/***
 * @Author: szf
 * @Date: 2023-05-13 15:33:55
 * @LastEditors: szf
 * @Description: 底盘状态调整
 * @FilePath: \RR-Chassis\Usercode\user_inc\chassis_state_management.h
 * @@WeChat:szf13373959031
 */
#ifndef CHASSIS_STATE_MANAGEMANT
#define CHASSIS_STATE_MANAGEMANT

#include "chassis_state_machine.h"
#include "chassis_perception.h"
#include "wtr_calculate.h"
#include "semphr.h"
#include "chassis_commen.h"

void StateManagemantTaskStart();

void ChassisInit();

void PIDInit();

void DJIRemoteControl();

void Automatic();
#endif
