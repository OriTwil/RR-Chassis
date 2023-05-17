/***
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-05-13 19:56:24
 * @LastEditors: szf
 * @Description: 主函数
 * @FilePath: \RR-Chassis\Usercode\user_inc\user_main.h
 * @@WeChat:szf13373959031
 */

#pragma once

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
#include "wtr_mavlink.h"
#include "wtr_time.h"
#include "wtr_uart.h"

#include "user_callback.h"
#include "user_calculate.h"
#include "user_config.h"

#include "chassis_perception.h"
#include "chassis_state_machine.h"
#include "chassis_communicate.h"
#include "chassis_servo.h"
#include "chassis_state_management.h"

#include "mavlink_msg_chassis_to_upper.h"
#include "mavlink_msg_control.h"
#include "mavlink_msg_posture.h"

void CtrlDataSender_Init(UART_HandleTypeDef *huart, mavlink_channel_t chan);
void BlinkLED();
