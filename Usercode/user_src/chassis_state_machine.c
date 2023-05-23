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
#include "chassis_state_management.h"

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
        switch (Robot_state.Chassis_state) {
            case Locked:
                if (islocked == false) // todo 第一次进入locked状态(不够简洁)
                {
                    pos_x_locked = mav_posture.pos_x;
                    pos_y_locked = mav_posture.pos_y;
                    pos_w_locked = mav_posture.zangle; // 记录下刚切换到Locked状态的坐标
                    islocked     = true;
                }
                SetChassisControlPosition(pos_x_locked, pos_y_locked, pos_w_locked, &Chassis_Control);                                                                      // 锁死在Locked位置
                SetChassisControlVelocity(PIDPosition(&Chassis_Pid.Pid_pos_x), PIDPosition(&Chassis_Pid.Pid_pos_y), PIDPosition(&Chassis_Pid.Pid_pos_w), &Chassis_Control); // 反馈控制底盘锁死在该位置

            case RemoteControl:
                islocked = false;

                vPortEnterCritical();
                SetChassisPosition(mav_posture.pos_x, mav_posture.pos_y, mav_posture.zangle, &Chassis_Position);                                                            // 更新底盘位置
                DeadBand((double)crl_speed.vx, (double)crl_speed.vy, &vx_deadbanded, &vy_deadbanded, 0.1);                                                                  // 死区控制 DJI遥控器摇杆
                SetChassisControlPosition(Chassis_Position.Chassis_Position_x, Chassis_Position.Chassis_Position_y, Chassis_Position.Chassis_Position_w, &Chassis_Control); // 没什么用，反正这个状态用不到PID
                SetChassisControlVelocity(vx_deadbanded, vy_deadbanded, crl_speed.vw, &Chassis_Control);                                                                    // 用摇杆控制底盘
                vPortExitCritical();

            case ComputerControl:
                islocked = false;
                vPortEnterCritical();
                SetChassisPosition(mav_posture.pos_x, mav_posture.pos_y, mav_posture.zangle, &Chassis_Position);                                                                                                               // 更新底盘位置
                SetChassisControlPosition(control.x_set, control.y_set, control.w_set, &Chassis_Control);                                                                                                                      // 上位机规划值作为伺服值
                SetChassisControlVelocity(control.vx_set + PIDPosition(&Chassis_Pid.Pid_pos_x), control.vy_set + PIDPosition(&Chassis_Pid.Pid_pos_y), control.vw_set + PIDPosition(&Chassis_Pid.Pid_pos_w), &Chassis_Control); // 上位机规划值作为伺服值
                vPortExitCritical();
        }

        vTaskDelayUntil(&PreviousWakeTime, 3);
    }
}

/**
 * @description: 开启线程
 * @date:
 * @return {void}
 */
void ChassisStateMachineTaskStart()
{
    osThreadDef(chassis, ChassisStateMachineTask, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(chassis), NULL);
}
