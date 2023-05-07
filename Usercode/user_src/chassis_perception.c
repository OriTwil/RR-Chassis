/*
 * @Author: szf
 * @Date: 2023-02-23 19:13:13
 * @LastEditTime: 2023-05-07 16:38:50
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \RR-Chassis\Usercode\user_src\chassis_perception.c
 * @WeChat:szf13373959031
 */
#include "chassis_perception.h"
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
#include "usermain.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "usercallback.h"
#include "usercalculate.h"
#include "stdio.h"

uint32_t test_pos[6] = {0};

/**
 * @description: 定位系统
 * @author: szf
 * @date:
 * @return {void}
 */
void ChassisPerceptionTask(void const *argument)
{
    // 码盘定位系统通过串口收信息
    HAL_UART_Receive_IT(&huart6, (uint8_t *)&ch, 1);

    for (;;) {
        mavlink_msg_posture_send_struct(MAVLINK_COMM_0,&mav_posture);
        osDelay(10);
    }
}

void PerceptionTaskStart(mavlink_controller_t *ctrl_data)
{
    osThreadDef(perception, ChassisPerceptionTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(perception), ctrl_data);
}

/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
