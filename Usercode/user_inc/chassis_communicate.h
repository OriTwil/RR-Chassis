/**
 * @author OriTwil
 * @brief 通信组件
 * @version 1.0
 * @date 2023.5.18
 * 
 */
#pragma once

#include "wtr_mavlink.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "user_main.h"

extern mavlink_control_t control; // 上位机规划后的控制信息
extern mavlink_posture_t mav_posture; // 定位系统的信息
// extern mavlink_controller_t ControllerData; //todo 遥控器
extern mavlink_channel_t CtrlDataSendChan;

void CommunicateTaskStart();
