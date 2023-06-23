/*
 * @Author: szf
 * @Date: 2023-02-23 19:01:45
 * @LastEditTime: 2023-05-13 20:38:42
 * @LastEditors: szf
 * @Description: 全向轮底盘
 * @FilePath: \RR-Chassis\Usercode\user_src\chassis_control_ow.c
 * @WeChat:szf13373959031
 */

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
#include <math.h>
#include "chassis_state_machine.h"
#include "chassis_communicate.h"
#include "chassis_commen.h"
#include "chassis_servo.h"
#include "chassis_remote_control.h"

CHASSIS_POSITION Chassis_Position;
CHASSIS_CONTROL Chassis_Control;
ROBOT_STATE Robot_state;
SPEED_RATIO Speed_ratio;

double vx_deadbanded = 0;
double vy_deadbanded = 0;
bool islocked        = false;

float pos_x_locked = 0;
float pos_y_locked = 0;
float pos_w_locked = 0;

/**
 * @description: 线程一：底盘状态机
 * @date:
 * @return {void}
 */
void ChassisStateMachineTask(void const *argument)
{
    // 初始化
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    // 解算，速度伺服
    for (;;) {
        // 拷贝数据，防止关中断或取互斥锁时间过长
        ROBOT_STATE Robot_state_temp = ReadRobotState(&Robot_state);
        vPortEnterCritical();
        mavlink_control_t control_temp = control;
        mavlink_posture_t posture_temp = mav_posture;
        vPortExitCritical();

        switch (Robot_state_temp.Chassis_state) {
            case Locked:
                if (islocked == false) // todo 第一次进入locked状态(不够简洁)
                {
                    pos_x_locked = posture_temp.pos_x;
                    pos_y_locked = posture_temp.pos_y;
                    pos_w_locked = posture_temp.zangle; // 记录下刚切换到Locked状态的坐标
                    islocked     = true;
                }
                SetChassisControlPosition(pos_x_locked, pos_y_locked, pos_w_locked, &Chassis_Control); // 锁死在Locked位置
                SetChassisControlVelocity(PIDPosition(&Chassis_Pid.Pid_pos_x),
                                          PIDPosition(&Chassis_Pid.Pid_pos_y),
                                          PIDPosition(&Chassis_Pid.Pid_pos_w),
                                          &Chassis_Control); // 反馈控制底盘锁死在该位置
                break;

            case RemoteControl:
                islocked = false;

                SetChassisPosition(posture_temp.pos_x, posture_temp.pos_y, posture_temp.zangle, &Chassis_Position); // 更新底盘位置

                Joystick_Control();
                // DJI_Control();
                vPortEnterCritical();
                DeadBand((double)crl_speed.vx, (double)crl_speed.vy, &vx_deadbanded, &vy_deadbanded, 0.1); // 死区控制 DJI遥控器摇杆
                // double vx_deadbanded = vx_deadbanded * cos(mav_posture.zangle * DEC) + vy_deadbanded * sin(mav_posture.zangle * DEC);
                // double vy_deadbanded = -vx_deadbanded * sin(mav_posture.zangle * DEC) + vy_deadbanded * cos(mav_posture.zangle * DEC);
                SetChassisControlVelocity(vx_deadbanded, vy_deadbanded, crl_speed.vw, &Chassis_Control); // 用摇杆控制底盘
                vPortExitCritical();
                break;
            case ComputerControl:
                islocked = false;
                SetChassisPosition(posture_temp.pos_x, posture_temp.pos_y, posture_temp.zangle, &Chassis_Position);      // 更新底盘位置
                SetChassisControlPosition(control_temp.x_set, control_temp.y_set, control_temp.w_set, &Chassis_Control); // 上位机规划值作为伺服值

                xSemaphoreTake(Chassis_Pid.xMutex_pid, (TickType_t)10);
                control_temp.vx_set = control_temp.vx_set + PIDPosition(&Chassis_Pid.Pid_pos_x);
                control_temp.vy_set = control_temp.vy_set + PIDPosition(&Chassis_Pid.Pid_pos_y);
                control_temp.vw_set = PIDPosition(&Chassis_Pid.Pid_pos_w);
                xSemaphoreGive(Chassis_Pid.xMutex_pid);
                control_temp = FrameTransform(&control_temp, &mav_posture);

                SetChassisControlVelocity(control_temp.vx_set,
                                          control_temp.vy_set,
                                          control_temp.vw_set,
                                          &Chassis_Control); // 上位机规划值作为伺服值
                break;
        }

        vTaskDelayUntil(&PreviousWakeTime, 5);
    }
}

void ChassisStateTestTask(void const *argument)
{
    vTaskDelay(20);
    // 解算，速度伺服
    for (;;) { // 上位机规划值作为伺服值

        vTaskDelay(5);
    }
}

/**
 * @description: 开启线程
 * @date:
 * @return {void}
 */
void ChassisStateMachineTaskStart()
{
    osThreadDef(chassis, ChassisStateMachineTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(chassis), NULL);

    // osThreadDef(chassis_test, ChassisStateTestTask, osPriorityNormal, 0, 2048);
    // osThreadCreate(osThread(chassis_test), NULL);
}

/**
 * @description: 切换底盘点位
 * @param target_chassis_point 目标点位(First_Point Second_Point Third_Point ...)
 * @return {void}
 */
void ChassisSwitchPoint(CHASSIS_POINT target_chassis_point, ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake(current_robot_state->xMutex_Robot, (TickType_t)10);
    current_robot_state->Chassis_point = target_chassis_point;
    xSemaphoreGive(current_robot_state->xMutex_Robot);
}

/**
 * @description: 切换底盘状态
 * @param target_chassis_state 目标状态(Locked RemoteControl ComputerControl)
 * @return {void}
 */
void ChassisSwitchState(CHASSIS_STATE target_chassis_state, ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake(current_robot_state->xMutex_Robot, (TickType_t)10);
    current_robot_state->Chassis_state = target_chassis_state;
    xSemaphoreGive(current_robot_state->xMutex_Robot);
}

void SpeedSwitchRatio(double target_speed_ratio_linear, double target_speed_ratio_angular, SPEED_RATIO* Speed_Ratio)
{
    xSemaphoreTake(Speed_ratio.xMutex_speed_ratio, portMAX_DELAY);
    Speed_Ratio->speed_ratio_angular = target_speed_ratio_angular;
    Speed_Ratio->speed_ratio_linear = target_speed_ratio_linear;
    xSemaphoreGive(Speed_ratio.xMutex_speed_ratio);
}

/**
 * @description: 更新底盘位置
 * @param position_ 底盘位置信息
 * @return {void}
 */
void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position)
{
    xSemaphoreTake(chassis_position->xMutex_position, (TickType_t)10);
    chassis_position->Chassis_Position_x = position_x;
    chassis_position->Chassis_Position_y = position_y;
    chassis_position->Chassis_Position_w = position_w;
    xSemaphoreGive(chassis_position->xMutex_position);
}

/**
 * @description: 设置底盘伺服速度值
 * @param vx_control 底盘速度伺服值
 * @return {void}
 */
void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_vx = vx_control;
    chassis_control->Chassis_Control_vy = vy_control;
    chassis_control->Chassis_Control_vw = vw_control;
    xSemaphoreGive(chassis_control->xMutex_control);
}

/**
 * @description: 设置底盘目标位置
 * @param _control 底盘目标位置
 * @return {void}
 */
void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control)
{
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control->Chassis_Control_x = x_control;
    chassis_control->Chassis_Control_y = y_control;
    chassis_control->Chassis_Control_w = w_control;
    xSemaphoreGive(chassis_control->xMutex_control);
}

ROBOT_STATE ReadRobotState(ROBOT_STATE *current_robot_state)
{
    ROBOT_STATE robot_state_temp;
    xSemaphoreTake(current_robot_state->xMutex_Robot, (TickType_t)10);
    robot_state_temp = *current_robot_state;
    xSemaphoreGive(current_robot_state->xMutex_Robot);
    return robot_state_temp;
}

CHASSIS_CONTROL ReadChassisControl(CHASSIS_CONTROL *chassis_control)
{
    CHASSIS_CONTROL chassis_control_temp;
    xSemaphoreTake(chassis_control->xMutex_control, (TickType_t)10);
    chassis_control_temp = *chassis_control;
    xSemaphoreGive(chassis_control->xMutex_control);
    return chassis_control_temp;
}

CHASSIS_POSITION ReadChassisPosition(CHASSIS_POSITION *chassis_position)
{
    CHASSIS_POSITION chassis_position_temp;
    xSemaphoreTake(chassis_position->xMutex_position, (TickType_t)10);
    chassis_position_temp = *chassis_position;
    xSemaphoreGive(chassis_position->xMutex_position);
    return chassis_position_temp;
}

void Joystick_Control()
{
    xSemaphoreTake(Speed_ratio.xMutex_speed_ratio, portMAX_DELAY);
    double speed_ratio_linear_temp  = Speed_ratio.speed_ratio_linear;
    double speed_ratio_angular_temp = Speed_ratio.speed_ratio_angular;
    xSemaphoreGive(Speed_ratio.xMutex_speed_ratio);
    crl_speed.vx = ReadJoystickRight_x(&Msg_joystick_air) * speed_ratio_linear_temp;
    crl_speed.vy = ReadJoystickRight_y(&Msg_joystick_air) * speed_ratio_linear_temp;
    crl_speed.vw = ReadJoystickLeft_x(&Msg_joystick_air) * speed_ratio_angular_temp;
}

/**
 * @description: 速度方向转换
 * @param
 * @return
 * @bug 转换方程还不确定
 */
mavlink_control_t FrameTransform(mavlink_control_t *control, mavlink_posture_t *posture)
{
    mavlink_control_t result;
    vPortEnterCritical();
    result.vx_set = control->vx_set * cos(posture->zangle * DEC) + control->vy_set * sin(posture->zangle * DEC);
    result.vy_set = -control->vx_set * sin(posture->zangle * DEC) + control->vy_set * cos(posture->zangle * DEC);
    result.vw_set = control->vw_set;
    result.w_set  = control->w_set;
    result.x_set  = control->x_set;
    result.y_set  = control->y_set;
    vPortExitCritical();
    return result;
}
