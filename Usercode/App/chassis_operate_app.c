/*
 * @OriTwil
 * @Date: 2023-05-11 16:08:17
 * @LastEditors: szf
 * @Description: 操作线程
 * @FilePath: \RR-Upper-Structure-A\UserCode\user_src\chassis_operate_app.c
 * @WeChat:szf13373959031
 */
#include "chassis_operate_app.h"
#include "chassis_communicate.h"
#include "chassis_state_machine.h"
#include "chassis_perception.h"
#include "chassis_servo.h"

#include "wtr_uart.h"

#include "chassis_config.h"
#include "chassis_commen.h"
#include "chassis_remote_control.h"
/**
 * @description: 操作线程
 * @todo 根据各个传感器、遥控器等设计操作手操作流程
 * @return {void}
 */
void StateManagemantTask(void const *argument)
{
    vTaskDelay(20);
    for (;;) {
        // 自动模式
        Automatic();
        // DJI遥控器控制模式
        // DJIRemoteControl();

        vTaskDelay(10);
    }
}

/**
 * @description: 开启线程
 * @return {void}
 */
void StateManagemantTaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(statemanagement), NULL);
}

/**
 * @description: 底盘初始化
 * @return {void}
 */
void ChassisInit()
{
    Robot_state.Chassis_point    = First_Point;
    Robot_state.Chassis_state    = RemoteControl;
    Robot_state.Perception_state = Receive;
    Robot_state.xMutex_Robot     = xSemaphoreCreateMutex();

    Chassis_Position.Chassis_Position_w = 0;
    Chassis_Position.Chassis_Position_x = 0;
    Chassis_Position.Chassis_Position_y = 0;
    Chassis_Position.Chassis_Angle_y    = 0;
    Chassis_Position.xMutex_position    = xSemaphoreCreateMutex();

    Chassis_Control.Chassis_Control_vw = 0;
    Chassis_Control.Chassis_Control_vx = 0;
    Chassis_Control.Chassis_Control_vy = 0;
    Chassis_Control.Chassis_Control_w  = 0;
    Chassis_Control.Chassis_Control_x  = 0;
    Chassis_Control.Chassis_Control_y  = 0;
    Chassis_Control.xMutex_control     = xSemaphoreCreateMutex();

    PIDInit();
    Chassis_Pid.xMutex_pid = xSemaphoreCreateMutex();

    Msg_joystick_air.xMutex_joystick_air                                   = xSemaphoreCreateMutex();
    msg_joystick_air_led.xMutex_joystick_air_led                           = xSemaphoreCreateMutex();
    msg_joystick_air_title_point.xMutex_joystick_air_dashboard_set_title   = xSemaphoreCreateMutex();
    msg_joystick_air_title_state.xMutex_joystick_air_dashboard_set_title   = xSemaphoreCreateMutex();
    msg_joystick_air_title_posture.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateMutex();
    msg_joystick_air_msg_point.xMutex_joystick_air_dashboard_set_msg       = xSemaphoreCreateMutex();
    msg_joystick_air_msg_state.xMutex_joystick_air_dashboard_set_msg       = xSemaphoreCreateMutex();
    msg_joystick_air_msg_posture.xMutex_joystick_air_dashboard_set_msg     = xSemaphoreCreateMutex();
    msg_joystick_air_delete.xMutex_joystick_air_dashboard_del              = xSemaphoreCreateMutex();
}

/**
 * @description: PID参数初始化
 * @return {void}
 */
void PIDInit()
{
    // 位置式pid参数设置
    Chassis_Pid.Pid_pos_w.Kp    = 40;
    Chassis_Pid.Pid_pos_w.Ki    = 0;
    Chassis_Pid.Pid_pos_w.Kd    = 0;
    Chassis_Pid.Pid_pos_w.limit = 0.5;

    Chassis_Pid.Pid_pos_x.Kp    = 5;
    Chassis_Pid.Pid_pos_x.Ki    = 0;
    Chassis_Pid.Pid_pos_x.Kd    = 0;
    Chassis_Pid.Pid_pos_x.limit = 0.5;

    Chassis_Pid.Pid_pos_y.Kp    = 5;
    Chassis_Pid.Pid_pos_y.Ki    = 0;
    Chassis_Pid.Pid_pos_y.Kd    = 0;
    Chassis_Pid.Pid_pos_y.limit = 0.5;
}

void DJIRemoteControl()
{
    vPortEnterCritical();
    if (Raw_Data.left == 1 /* 判断按键1是否按下 */) {
        mav_posture.point  = 1;
        chassis_data.point = 1;
    }
    if (Raw_Data.left == 2 /* 判断按键2是否按下 */) {
        mav_posture.point  = 2;
        chassis_data.point = 2;
    }
    if (Raw_Data.left == 3 /* 判断按键3是否按下 */) {
        mav_posture.point  = 3;
        chassis_data.point = 3;
    }
    vPortExitCritical();

    ChassisSwitchState(RemoteControl, &Robot_state);
}

void Automatic()
{

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossLeft)) {
        // Code for Btn_LeftCrossLeft
        ChassisSwitchPoint(First_Point, &Robot_state);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossRight)) {
        // Code for Btn_LeftCrossRight
        ChassisSwitchPoint(Second_Point, &Robot_state);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossMid)) {
        // Code for Btn_LeftCrossMid
        ChassisSwitchPoint(Third_Point, &Robot_state);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossUp)) {
        // Code for Btn_RightCrossUp
        ChassisSwitchPoint(Fourth_Point, &Robot_state);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossDown)) {
        // Code for Btn_RightCrossDown
        ChassisSwitchPoint(Fifth_Point, &Robot_state);
    }

    if (ReadJoystickSwitchs(&Msg_joystick_air, Left_switch) == 0) {
        ChassisSwitchState(Locked, &Robot_state);
    }

    if (ReadJoystickSwitchs(&Msg_joystick_air, Left_switch) == 1) {
        if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn2)) {
            ChassisSwitchState(RemoteControl, &Robot_state);
        }
        if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn3)) {
            ChassisSwitchState(ComputerControl, &Robot_state);
        }
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_JoystickL)) {
        vPortEnterCritical();
        mav_posture.point = First_Point;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_JoystickR)) {
        vPortEnterCritical();
        mav_posture.point = Second_Point;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_KnobL)) {
        vPortEnterCritical();
        mav_posture.point = Third_Point;
        vPortExitCritical();
    }
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_KnobR)) {
        vPortEnterCritical();
        mav_posture.point = Fourth_Point;
        vPortExitCritical();
    }
}

// todo 写一个从地图坐标系到底盘坐标系的转换函数