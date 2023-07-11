/***
 * @Author: szf
 * @Date: 2023-02-22 12:05:56
 * @LastEditTime: 2023-02-22 12:25:39
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usercallback.h
 * @@WeChat:szf13373959031
 */
#ifndef __WTR_CALLBACK_H__
#define __WTR_CALLBACK_H__

#include "chassis_start.h"
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

extern int16_t crldata[4];
extern int led_count;
extern int led_ops_count;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif
