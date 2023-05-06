/*
 * @Author: szf
 * @Date: 2023-02-22 12:04:21
 * @LastEditTime: 2023-05-06 16:51:08
 * @LastEditors: szf
 * @Description: 运动学逆解算及PID计算函数
 * @FilePath: \RR-Chassis\Usercode\user_src\usercalculate.c
 * @WeChat:szf13373959031
 */

#include "usermain.h"
#include "usercalculate.h"
#include <math.h>

double moter_speed[4];

/**
 * @description:三轮底盘逆解算（第一种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateThreeWheels(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w)
{
    moter_speed[0] = (-v_x * sin(30 * DEC) - v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * pi * r_wheel);
    moter_speed[1] = (+v_x + v_w * r_underpan_3) / (2 * pi * r_wheel);
    moter_speed[2] = (-v_x * sin(30 * DEC) + v_y * cos(30 * DEC) + v_w * r_underpan_3) / (2 * pi * r_wheel);
}

/**
 * @description:三轮底盘逆解算（第二种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateThreeWheels_(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w)
{
    moter_speed[2] = (-v_y * sin(30 * DEC) + v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
    moter_speed[1] = (+v_y + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
    moter_speed[0] = (-v_y * sin(30 * DEC) - v_x * cos(30 * DEC) + v_w * r_underpan_3) * 60 / (2 * pi * r_wheel) * 19;
}

/**
 * @description:四轮底盘逆解算（第一种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateFourWheels(double *moter_speed,
                 double v_x,
                 double v_y,
                 double v_w)
{
    moter_speed[0] = (v_y + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[1] = (-v_x + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[2] = (-v_y + v_w * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[3] = (v_x + v_w * r_underpan_4) / (2 * pi * r_wheel);
}

/**
 * @description:四轮底盘逆解算（第二种方向）
 * @author: szf
 * @date:
 * @return {void}
 */
void CalculateFourWheels_(double *moter_speed,
                   double v_x,
                   double v_y,
                   double v_w)
{
    moter_speed[0] = (vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[1] = (-vx * sqrt(2) + vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[2] = (-vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
    moter_speed[3] = (vx * sqrt(2) - vy * sqrt(2) + vw * r_underpan_4) / (2 * pi * r_wheel);
}

/**
 * @description: 增量式PID
 * @param {PID} *vPID
 * @param {float} processValue
 * @return {*}
 */
void PIDIncremental(PID_Incremwntal *vPID, float processValue)
{
    float thisError;
    float increment;
    float pError, dError, iError;

    thisError = vPID->setpoint - processValue; // 当前误差等于设定值减去当前值
    // 计算公式中除系数外的三个 乘数
    pError = thisError - vPID->lasterror; // 两次偏差差值err(k)-err(k-1)
    iError = thisError;
    dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

    increment = vPID->Kp * pError + vPID->Ki * iError + vPID->Kd * dError; // 增量计算

    vPID->preerror  = vPID->lasterror; // 存放偏差用于下次运算
    vPID->lasterror = thisError;

    vPID->result += increment; // 结果是上次结果 加上本次增量
    vPID->result = range(vPID->result, -vPID->limit, vPID->limit);
    // if(vPID->result > 0.6)
    // {
    //     vPID->result = 0.6;
    // }
    // if (vPID->result < -0.6)
    // {
    //     vPID->result = -0.6;
    // }//输出限幅
}

/**
 * @description: 位置式PID
 * @param {PIDType} *p
 * @return {*}
 */
float PIDPosition(PID_Pos *p)
{
    float pe, ie, de;
    float out = 0;

    // 计算当前误差
    p->e0 = p->target - p->feedback;

    // 误差积分
    p->eSum += p->e0;

    // 误差微分
    de = p->e0 - p->e1;

    pe = p->e0;
    ie = p->eSum;

    p->e1 = p->e0;

    out = pe * (p->Kp) + ie * (p->Ki) + de * (p->Kd);
    // 输出限幅
    out = range(out, -p->limit, p->limit);
    return out;
}

/**
 * @description: 速度方向转换
 * @param 
 * @return 
 * @bug 转换方程还不确定
 */
mavlink_control_set_t FrameTransform(mavlink_control_set_t *control,mavlink_posture_t *posture)
{
    mavlink_control_set_t result;
    result.vx_set = control->vx_set * cos(posture->zangle) + control->vy_set * sin(posture->zangle);
    result.vy_set = -control->vx_set * sin(posture->zangle) + control->vy_set * cos(posture->zangle);
    return result;
}