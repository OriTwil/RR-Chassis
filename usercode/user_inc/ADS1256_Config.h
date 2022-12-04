/*
 * @Author: szf01 2176529058@qq.com
 * @Date: 2022-12-03 20:06:20
 * @LastEditors: szf01 2176529058@qq.com
 * @LastEditTime: 2022-12-03 20:48:15
 * @FilePath: /underpan/usercode/user_inc/ADS1256_Config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file ADS1256_Config.h
 * @author TITH (1023515576@qq.com)
 * @brief ADS1256的驱动程序配置文件，不需要include此文件
 * @version 0.2
 * @date 2022-01-11
 */

#pragma once

#include "ADS1256.h"

// 引脚配置

#define ADS1256_DRDY_GPIOx GPIOA
#define ADS1256_DRDY_GPIO_Pin GPIO_PIN_4

#define ADS1256_Reset_GPIOx GPIOB
#define ADS1256_DRDY_GPIOx     GPIOA
#define ADS1256_DRDY_GPIO_Pin  GPIO_PIN_4

#define ADS1256_Reset_GPIOx    GPIOB

#define ADS1256_Reset_GPIO_Pin GPIO_PIN_2

// 控制是否启用NSS片选

// #define ADS1256_NSS_Enable
#ifdef ADS1256_NSS_Enable

#define ADS1256_NSS_GPIOx
#define ADS1256_NSS_GPIO_Pin
#endif // 有片选引脚可以打开
