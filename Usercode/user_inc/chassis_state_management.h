/***
 * @Author: szf
 * @Date: 2023-05-13 15:33:55
 * @LastEditors: szf
 * @Description: 底盘状态调整
 * @FilePath: \RR-Chassis\Usercode\user_inc\chassis_state_management.h
 * @@WeChat:szf13373959031
 */
#ifndef CHASSIS_STATE_MANAGEMANT
#define CHASSIS_STATE_MANAGEMANT

#include "user_main.h"
#include "chassis_state_machine.h"
#include "chassis_perception.h"
#include "user_calculate.h"
#include "semphr.h"

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

void StateManagemantTaskStart();

void ChassisInit();

void PIDInit();

void ChassisSwitchPoint(CHASSIS_POINT target_chassis_point, ROBOT_STATE *current_robot_state);

void ChassisSwitchState(CHASSIS_STATE target_chassis_state, ROBOT_STATE *current_robot_state);

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, ROBOT_STATE *current_robot_state);

void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid);

void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid);

void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position);

void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control);

void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control);

extern ROBOT_STATE Robot_state;
extern CHASSIS_PID Chassis_Pid;
extern CHASSIS_POSITION Chassis_Position;
extern CHASSIS_CONTROL Chassis_Control;
extern TaskHandle_t g_stateManagementTaskHandle;

#endif