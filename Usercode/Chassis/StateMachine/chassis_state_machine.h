/***
 * @Author: szf
 * @Date: 2023-02-23 19:03:31
 * @LastEditTime: 2023-05-13 19:56:07
 * @LastEditors: szf
 * @Description: 状态机
 * @@WeChat:szf13373959031
 */
#pragma once

#include "can.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "chassis_state_management.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "chassis_start.h"
#include "wtr_calculate.h"

void ChassisStateMachineTaskStart();

// extern CHASSIS_STATE Chassis_state;