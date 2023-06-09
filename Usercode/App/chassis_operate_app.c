/*
 * @OriTwil
 * @Date: 2023-05-11 16:08:17
 * @LastEditors: szf
 * @Description: 操作手操作线程
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

#define LOW_SPEED_STATE  (ReadSpeedRatio(&Speed_ratio).speed_ratio_linear < 0.5)
#define HIGH_SPEED_STATE (ReadSpeedRatio(&Speed_ratio).speed_ratio_linear >= 0.5)

float test_ratio;
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

        vTaskDelay(100);
    }
}

/**
 * @description: 开启线程
 * @return {void}
 */
void StateManagemantTaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityBelowNormal, 0, 2048);
    osThreadCreate(osThread(statemanagement), NULL);
}

/**
 * @description: 底盘初始化
 * @return {void}
 */
void ChassisInit()
{
    Robot_state.Chassis_point    = Zero_Point;
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

    Speed_ratio.speed_ratio_angular = 1.5;
    Speed_ratio.speed_ratio_linear  = 1.0;
    Speed_ratio.xMutex_speed_ratio  = xSemaphoreCreateMutex();

    Msg_joystick_air.xMutex_joystick_air                                   = xSemaphoreCreateMutex();
    msg_joystick_air_led.xMutex_joystick_air_led                           = xSemaphoreCreateMutex();
    msg_joystick_air_title_point.xMutex_joystick_air_dashboard_set_title   = xSemaphoreCreateMutex();
    msg_joystick_air_title_state.xMutex_joystick_air_dashboard_set_title   = xSemaphoreCreateMutex();
    msg_joystick_air_title_posture.xMutex_joystick_air_dashboard_set_title = xSemaphoreCreateMutex();
    msg_joystick_air_title_knob_r.xMutex_joystick_air_dashboard_set_title  = xSemaphoreCreateMutex();

    msg_joystick_air_msg_point.xMutex_joystick_air_dashboard_set_msg   = xSemaphoreCreateMutex();
    msg_joystick_air_msg_state.xMutex_joystick_air_dashboard_set_msg   = xSemaphoreCreateMutex();
    msg_joystick_air_msg_posture.xMutex_joystick_air_dashboard_set_msg = xSemaphoreCreateMutex();
    msg_joystick_air_delete.xMutex_joystick_air_dashboard_del          = xSemaphoreCreateMutex();
    msg_joystick_air_msg_knob_r.xMutex_joystick_air_dashboard_set_msg  = xSemaphoreCreateMutex();

    Baffle.position_servo_ref_baffle = 0;
    Baffle.xMutex_baffle             = xSemaphoreCreateMutex();

    Control.control_w          = 0;
    Control.micro_adjustment_w = 0;
    Control.xMutex_w           = xSemaphoreCreateMutex();

    Fire_Target.Fire_number   = Fifth_Target;
    Fire_Target.xMutex_target = xSemaphoreCreateMutex();
}

/**
 * @description: PID参数初始化
 * @return {void}
 */
void PIDInit()
{
    // 位置式pid参数设置
    Chassis_Pid.Pid_pos_w.Kp    = 0.1;
    Chassis_Pid.Pid_pos_w.Ki    = 0;
    Chassis_Pid.Pid_pos_w.Kd    = 0;
    Chassis_Pid.Pid_pos_w.limit = 1.2;

    Chassis_Pid.Pid_pos_x.Kp    = 5;
    Chassis_Pid.Pid_pos_x.Ki    = 0.0001;
    Chassis_Pid.Pid_pos_x.Kd    = 0;
    Chassis_Pid.Pid_pos_x.limit = 0.5;

    Chassis_Pid.Pid_pos_y.Kp    = 5;
    Chassis_Pid.Pid_pos_y.Ki    = 0.0001;
    Chassis_Pid.Pid_pos_y.Kd    = 0;
    Chassis_Pid.Pid_pos_y.limit = 0.5;
}

/**
 * @description: DJI遥控器控制
 * @return {void}
 */
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


/**
 * @description: 自制遥控器操作
 * @return {void}
 */
void Automatic()
{

    if (ReadJoystickSwitchs(&Msg_joystick_air, Right_switch) == 1) {
        SpeedSwitchRatio(1.6, 1.0, &Speed_ratio);
    } else if (ReadJoystickSwitchs(&Msg_joystick_air, Right_switch) == 0) {
        SpeedSwitchRatio(0.3, 0.5, &Speed_ratio);
    }

    // 切换手动自动模式
    if (ReadJoystickSwitchs(&Msg_joystick_air, Left_switch) == 0) {
        ChassisSwitchState(AutoControl, &Robot_state);
    }

    if (ReadJoystickSwitchs(&Msg_joystick_air, Left_switch) == 1) {
        ChassisSwitchState(RemoteControl, &Robot_state);
    }

    // 重定位
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn3)) {
        // HAL_UART_Transmit(&huart_OPS, "ACT0", 4, 50);
        Update_X(1740.0);
        vTaskDelay(10);
        Update_Y(5755.0);
        vTaskDelay(10);
        Update_A(90.0);
        vTaskDelay(10);

        vPortEnterCritical();
        mav_posture.point = Eighth_Point;
        control.x_set     = 1.74;
        control.y_set     = 5.755;
        vPortExitCritical();

        SetBaffleRef(1, &Baffle);
        // vTaskDelay(100);
    }

    // 切换点位
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn4)) {
        vPortEnterCritical();
        mav_posture.point = First_Point;
        vPortExitCritical();
        vTaskDelay(1500);
        SetChassisW(90.0, &Control);
        SetBaffleRef(BAFFLE_UP, &Baffle);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn5)) {
        vPortEnterCritical();
        mav_posture.point = Second_Point;
        vPortExitCritical();
        vTaskDelay(1500);
        SetChassisW(180.0, &Control);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_RightCrossUp)) {
        // SetChassisW(180.0, &Control);
        // vTaskDelay(2000);
        FireSwitchTarget(Seventh_Target, &Fire_Target);
        vTaskDelay(1500);
        vPortEnterCritical();
        mav_posture.point = Third_Point;
        vPortExitCritical();
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_RightCrossLeft)) {
        // SetChassisW(180.0, &Control);
        // vTaskDelay(2000);
        FireSwitchTarget(Seventh_Target, &Fire_Target);
        vTaskDelay(1500);
        vPortEnterCritical();
        mav_posture.point = Fourth_Point;
        vPortExitCritical();
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_RightCrossMid)) {
        vPortEnterCritical();
        mav_posture.point = Fifth_Point;
        vPortExitCritical();
        // vTaskDelay(1000);
        // SetChassisW(0.0, &Control);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_RightCrossRight)) {
        vPortEnterCritical();
        mav_posture.point = Sixth_Point;
        vPortExitCritical();
        SetBaffleRef(BAFFLE_UP, &Baffle);
        SetChassisW(90.0, &Control);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_RightCrossDown)) {
        vPortEnterCritical();
        mav_posture.point = Seventh_Point;
        vPortExitCritical();
        SetBaffleRef(BAFFLE_UP, &Baffle);
        SetChassisW(90.0, &Control);
    }

    // 切换目标柱子
    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossUp)) {
        FireSwitchTarget(First_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossRight)) {
        FireSwitchTarget(Second_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossMid)) {
        FireSwitchTarget(Third_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossLeft)) {
        FireSwitchTarget(Fourth_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_LeftCrossDown)) {
        FireSwitchTarget(Fifth_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn0)) {
        FireSwitchTarget(Sixth_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn1)) {
        FireSwitchTarget(Seventh_Target, &Fire_Target);
    }

    if (ReadJoystickButtons(&Msg_joystick_air, Btn_Btn2)) {
        FireSwitchTarget(Eighth_Target, &Fire_Target);
    }
}