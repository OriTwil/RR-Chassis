/*** 
 * @Author: szf
 * @Date: 2023-02-23 19:03:31
 * @LastEditTime: 2023-05-13 19:56:07
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \RR-Chassis\Usercode\user_inc\chassis_control_ow.h
 * @@WeChat:szf13373959031
 */
#ifndef OW_CHASSIS_CONTROL_H
#define OW_CHASSIS_CONTROL_H

#include "user_main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h" 
#include <math.h>
#include "chassis_control.h"
#include "chassis_state_management.h"

void OwChassisTaskStart(mavlink_controller_t *ctrl_data);
void PIDInit();
void PIDUpdate();
void PIDUpdateLocked(float pos_x,float pos_y,float angle_z);



// extern CHASSIS_STATE Chassis_state;
#endif