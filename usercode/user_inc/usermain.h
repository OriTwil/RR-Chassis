/*
 * @Author: szf01 2176529058@qq.com
 * @Date: 2022-12-03 20:06:20
 * @LastEditors: szf01 2176529058@qq.com
 * @LastEditTime: 2022-12-04 13:10:48
 * @FilePath: /underpan/usercode/user_inc/usermain.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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

// 宏定义计算中常数及底盘数据
#define pi           3.1415926535898
#define DEC          (pi / 180)
#define r_underpan_3 0.1934
#define r_underpan_4 0.25
#define r_wheel      0.076

// 声明运动学逆解函数
void calculate_4(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w);

void calculate_3(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w);
void calculate_3_2(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w);
/*定义位置环PID结构体*/
typedef struct
{
    float setpoint; //设定值
    float Kp;       //比例系数
    float Kd;       //积分系数
    float Ki;       //微分系数

    float lasterror; //前一拍偏差
    float preerror;  //前两拍偏差
    float result;    //输出值
} PID;

// 定义数组，分别存放四个轮子对应电机的速度
double moter_speed[4];

static uint8_t ch[1];

static union {
    uint8_t data[24];
    float ActVal[6];
} posture;

static uint8_t count = 0;

static uint8_t i = 0;

// mavlink_speed_t msg_receive;
mavlink_control_set_t control;

mavlink_speed_control_status_t v_state;

mavlink_posture_t mav_posture;

PID pid_pos_x;
PID pid_pos_y;
PID pid_vel_w;

#endif