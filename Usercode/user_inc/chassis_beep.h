/*** 
 * @Author: szf
 * @Date: 2023-02-23 20:29:04
 * @LastEditTime: 2023-02-23 20:29:54
 * @LastEditors: szf
 * @Description: 
 * @FilePath: \ER\Usercode\user_inc\beep.h
 * @@WeChat:szf13373959031
 */
/**
 * @file beep.h
 * @author TITH (1023515576@qq.com)
 * @brief 蜂鸣器
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

#define BeepPinGPIOx GPIOD
#define BeepPinGPIO_Pin GPIO_PIN_7

void Beep_Init();
void Beep();
void BeepSet(bool is_on);
void BeepFromISR();
