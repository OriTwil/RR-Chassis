/***
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-05-13 19:56:24
 * @LastEditors: szf
 * @Description: 主函数
 * @FilePath: \RR-Chassis\Usercode\user_inc\chassis_start.h
 * @@WeChat:szf13373959031
 */

#ifndef __CHASSIS_START_H__
#define __CHASSIS_START_H__

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
#include "wtr_calculate.h"

#include "chassis_config.h"

#include "chassis_perception.h"
#include "chassis_state_machine.h"
#include "chassis_communicate.h"
#include "chassis_servo.h"

#include "mavlink_msg_chassis_to_upper.h"
#include "mavlink_msg_control.h"
#include "mavlink_msg_posture.h"

void BlinkLED();

#endif