/**
 * @author OriTwil
 * @brief 通信组件
 * @version 1.0
 * @date 2023.5.18
 *
 */
#ifndef __CHASSIS_COMMUNICATE_H__
#define __CHASSIS_COMMUNICATE_H__

#include "wtr_mavlink.h"
#include "cmsis_os.h"
#include "usart.h"

// 按键通知值定义
#define BUTTON1_NOTIFICATION (1 << 0)
#define BUTTON2_NOTIFICATION (1 << 1)
#define BUTTON3_NOTIFICATION (1 << 2)

extern mavlink_control_t control;     // 上位机规划后的控制信息
extern mavlink_posture_t mav_posture; // 定位系统的信息
// extern mavlink_controller_t ControllerData; //todo 遥控器
extern mavlink_chassis_to_upper_t chassis_data;               

void CommunicateTaskStart();
void CommunicateInit();

#endif
