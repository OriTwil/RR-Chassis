/***
 * @Author: szf
 * @Date: 2023-02-22 12:06:17
 * @LastEditTime: 2023-02-22 12:30:10
 * @LastEditors: szf
 * @Description:
 * @FilePath: \ER\Usercode\user_inc\usercalculate.h
 * @@WeChat:szf13373959031
 */
#ifndef _USERCALCULATE_H__
#define _USERCALCULATE_H__

#include "usermain.h"
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

#define rotate_ratio 0.3615 // (Width + Length)/2
#define wheel_rpm_ratio 2387.324 // 

/*定义增量式PID结构体*/
typedef struct
{
    float setpoint; // 设定值
    float Kp;       // 比例系数
    float Kd;       // 积分系数
    float Ki;       // 微分系数

    float lasterror; // 前一拍偏差
    float preerror;  // 前两拍偏差
    float result;    // 输出值
    float limit;     // 输出限幅
} PID_Incremwntal;

/*定义位置式PID结构体*/
typedef struct {
    float limit;    // 输出限幅
    float target;   // 目标量
    float feedback; // 反馈量

    float Kp;
    float Ki;
    float Kd;
    float eSum;
    float e0; // 当前误差
    float e1; // 上一次误差
} PID_Pos;

// 定义数组，分别存放四个轮子对应电机的速度
extern double moter_speed[4];
// 声明运动学逆解函数
void CalculateFourWheels(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w);

void CalculateThreeWheels_(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w);
void CalculateThreeWheels(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w);
void CalculateFourWheels(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w);
void PIDIncremental(PID_Incremwntal *vPID, float processValue);
float PIDPosition(PID_Pos *p);
void CalculateFourMecanumWheels(double *moter_speed,double vx,double vy,double vw);

mavlink_control_set_t FrameTransform(mavlink_control_set_t *control,mavlink_posture_t *posture);


#endif