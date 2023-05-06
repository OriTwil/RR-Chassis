/*** 
 * @Author: szf
 * @Date: 2023-02-22 11:56:47
 * @LastEditTime: 2023-05-06 17:11:05
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \RR-Chassis\Usercode\user_inc\usermain.h
 * @@WeChat:szf13373959031
 */


#ifndef USERMAIN_H
#define USERMAIN_H

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include "mavlink_msg_speed_control_set.h"
#include "mavlink_msg_speed_control_status.h"
#include "mavlink_msg_control_set.h"
#include "usercallback.h"
#include "usercalculate.h"
#include "chassis_control.h"
#include "chassis_driver.h"
#include "chassis_perception.h"
#include "ow_chassis_control.h"
#include "ctrl_data_sender.h"
#include "beep.h"
#include "CLI.h"

// 宏定义计算中常数及底盘数据
#define pi           3.1415926535898
#define DEC          (pi / 180)
#define r_underpan_3 0.1934
#define r_underpan_4 0.25
#define r_wheel      0.076
// 限幅
#define max(a, b)      (a > b ? a : b)
#define min(a, b)      (a < b ? a : b)
#define range(x, a, b) (min(max(x, a), b))

// static uint8_t i = 0;
extern uint8_t i; // 不确定是不是static

extern mavlink_control_set_t control;
extern mavlink_speed_control_status_t v_state;
extern mavlink_posture_t mav_posture;

extern PID_Incremwntal pid_pos_x;
extern PID_Incremwntal pid_pos_y;
extern PID_Incremwntal pid_vel_w;

extern PID_Pos pid_pos_w_pos;
extern PID_Pos pid_pos_x_pos;
extern PID_Pos pid_pos_y_pos;

#endif