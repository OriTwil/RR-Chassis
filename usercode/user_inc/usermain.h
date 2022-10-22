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
// #include "mavlink_msg_speed.h"
#include "mavlink_msg_speed_control_set.h"
#include "mavlink_msg_speed_control_status.h"

//宏定义计算中常数及底盘数据
#define pi 3.1415926535898
#define DEC (pi/180)
#define r_underpan_3 0.1934
#define r_underpan_4 0.25
#define r_wheel 0.076

//声明运动学逆解函数
void calculate_4(double * moter_speed,
               double v_x,
               double v_y,
               double v_w);

void calculate_3(double * moter_speed,
               double v_x,
               double v_y,
               double v_w);

//定义数组，分别存放四个轮子对应电机的速度
double moter_speed [4];
//定义一个接收消息
// mavlink_speed_t msg_receive;
mavlink_speed_control_set_t v_set;
mavlink_speed_control_status_t v_state;

#endif