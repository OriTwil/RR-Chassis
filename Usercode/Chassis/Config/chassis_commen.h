#ifndef __CHASSIS_COMMEN_H__
#define __CHASSIS_COMMEN_H__

// 数据结构
typedef enum {
    Receive,
    Transmit
} PERCEPTION_STATE;

typedef enum {
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
    ComputerControl
} CHASSIS_STATE;

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
    SemaphoreHandle_t xMutex_Robot;
    CHASSIS_POINT Chassis_point;
} ROBOT_STATE;

#endif