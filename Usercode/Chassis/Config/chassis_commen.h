/*** 
 * @Author: OriTwil
 * @Description: 数据结构文件
 * @FilePath: \ER\Usercode\user_inc\useful_constant.h
 * @@WeChat:szf13373959031
 */
#ifndef __COMMEN_H__
#define __COMMEN_H__

#include "wtr_calculate.h"
typedef enum {
    Receive,
    Transmit
} PERCEPTION_STATE;

typedef enum {
    Zero_Point,
    First_Point,
    Second_Point,
    Third_Point,
    Fourth_Point,
    Fifth_Point,
    Sixth_Point,
    Seventh_Point,
    Eighth_Point,
    Ninth_Point,
    Tenth_Point
} CHASSIS_POINT;

typedef enum {
    Locked,
    RemoteControl,
    AutoControl
} CHASSIS_STATE;

typedef enum {
    First_Target = 1,
    Second_Target,
    Third_Target,
    Fourth_Target,
    Fifth_Target,
    Sixth_Target,
    Seventh_Target,
    Eighth_Target
} FIRE_NUMBER;

typedef __IO struct
{
    /* data */
    PID_Pos Pid_pos_w;
    PID_Pos Pid_pos_x;
    PID_Pos Pid_pos_y;
    SemaphoreHandle_t xMutex_pid;
} CHASSIS_PID;

typedef __IO struct
{
    float Chassis_Position_x;
    float Chassis_Position_y;
    float Chassis_Position_w;
    float Chassis_Angle_y; // 俯仰
    SemaphoreHandle_t xMutex_position;
} CHASSIS_POSITION;

typedef __IO struct
{
    float Chassis_Control_vx;
    float Chassis_Control_vy;
    float Chassis_Control_vw;
    float Chassis_Control_x;
    float Chassis_Control_y;
    float Chassis_Control_w;
    SemaphoreHandle_t xMutex_control;
} CHASSIS_CONTROL;

typedef __IO struct {
    PERCEPTION_STATE Perception_state;
    CHASSIS_STATE Chassis_state;
    CHASSIS_POINT Chassis_point;
    SemaphoreHandle_t xMutex_Robot;
} ROBOT_STATE;

typedef __IO struct
{
    /* data */
    float position_servo_ref_baffle;
    SemaphoreHandle_t xMutex_baffle;
} BAFFLE;

typedef __IO struct
{
    FIRE_NUMBER Fire_number;
    SemaphoreHandle_t xMutex_target;
}FIRE_TARGET;

typedef struct
{
    mavlink_joystick_air_t msg_joystick_air;
    SemaphoreHandle_t xMutex_joystick_air;
} JOYSTICK_AIR;
typedef struct
{
    mavlink_joystick_air_led_t msg_joystick_air_led;
    SemaphoreHandle_t xMutex_joystick_air_led;
} JOYSTICK_AIR_LED;

typedef struct
{
    mavlink_joystick_air_dashboard_set_title_t msg_joystick_air_dashboard_set_title;
    SemaphoreHandle_t xMutex_joystick_air_dashboard_set_title;
} JOYSTICK_AIR_DASHBOARD_SET_TITLE;

typedef struct
{
    mavlink_joystick_air_dashboard_set_msg_t msg_joystick_air_dashboard_set_msg;
    SemaphoreHandle_t xMutex_joystick_air_dashboard_set_msg;
} JOYSTICK_AIR_DASHBOARD_SET_MSG;

typedef struct
{
    mavlink_joystick_air_dashboard_del_t msg_joystick_air_dashboard_del;
    SemaphoreHandle_t xMutex_joystick_air_dashboard_del;
} JOYSTICK_AIR_DASHBOARD_DELETE;

// joystick
typedef enum {
    Left_switch = 0,
    Right_switch
} SWITCHS;

typedef __IO struct
{
    /* data */
    double speed_ratio_linear;
    double speed_ratio_angular;
    SemaphoreHandle_t xMutex_speed_ratio;
} SPEED_RATIO;

typedef __IO struct
{
    /* data */
    float control_w;
    float micro_adjustment_w;
    SemaphoreHandle_t xMutex_w;
}CONTROL;

typedef enum {
    Btn_LeftCrossUp     = 2,
    Btn_LeftCrossDown   = 1,
    Btn_LeftCrossLeft   = 13,
    Btn_LeftCrossRight  = 5,
    Btn_LeftCrossMid    = 9,
    Btn_RightCrossUp    = 4,
    Btn_RightCrossDown  = 3,
    Btn_RightCrossLeft  = 7,
    Btn_RightCrossRight = 15,
    Btn_RightCrossMid   = 11,
    Btn_Btn0            = 6,
    Btn_Btn1            = 10,
    Btn_Btn2            = 14,
    Btn_Btn3            = 8,
    Btn_Btn4            = 12,
    Btn_Btn5            = 16,
    Btn_KnobL           = 17,
    Btn_KnobR           = 18,
    Btn_JoystickL       = 19,
    Btn_JoystickR       = 20,
} KEYS;

#endif