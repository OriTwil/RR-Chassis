/*
 * @OriTwil
 * @Date: 2023-05-11 16:08:17
 * @LastEditTime: 2023-05-11 16:19:38
 * @LastEditors: szf
 * @Description: 任务切换线程
 * @FilePath: \RR-Upper-Structure-A\UserCode\user_src\chassis_state_management.c
 * @WeChat:szf13373959031
 */
#include "chassis_state_management.h"

ROBOT_STATE Robot_state; 
CHASSIS_PID Chassis_Pid;
CHASSIS_POSITION Chassis_Position;
CHASSIS_CONTROL Chassis_Control;

/**
 * @description: 操作线程
 * @todo 根据各个传感器、遥控器等设计操作手操作流程
 * @return {void}
 */
void StateManagemantTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 5);
    }
}

/**
 * @description: 测试
 * @return {void}
 */
void StateManagemantTestTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 5);
    }
}

/**
 * @description: 开启线程
 * @return {void}
 */
void StateManagemanttaskStart()
{

    osThreadDef(statemanagement, StateManagemantTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(statemanagement), NULL);

    // osThreadDef(statemanagementtest,StateManagemantTestTask,osPriorityBelowNormal,0,512);
    // osThreadCreate(osThread(statemanagementtest),NULL);
}

/**
 * @description: 底盘初始化
 * @return {void}
 */
void ChassisInit()
{
    Robot_state.Chassis_point    = First_Point;
    Robot_state.Chassis_state    = Locked;
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

/**
 * @description: 切换底盘点位
 * @param target_chassis_point 目标点位(First_Point Second_Point Third_Point ...)
 * @return {void}
 */
void ChassisSwsitchPoint(CHASSIS_POINT target_chassis_point, ROBOT_STATE *current_robot_state)
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

/**
 * @description: 切换传感器状态
 * @param target_perception_state 目标状态(Receive Transmit)
 * @return {void}
 */
void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, ROBOT_STATE *current_robot_state)
{
    xSemaphoreTake(current_robot_state->xMutex_Robot, (TickType_t)10);
    current_robot_state->Perception_state = target_perception_state;
    xSemaphoreGive(current_robot_state->xMutex_Robot);
}

/**
 * @description: 设置PID目标值
 * @param target_ PID目标值
 * @return {void}
 */
void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTake(chassis_pid->xMutex_pid, (TickType_t)10);
    // 位置式PID
    chassis_pid->Pid_pos_w.target = target_w;
    chassis_pid->Pid_pos_x.target = target_x;
    chassis_pid->Pid_pos_y.target = target_y;
    xSemaphoreGive(chassis_pid->xMutex_pid);
}

/**
 * @description: 设置PID反馈值
 * @param feedback_ PID反馈值
 * @return {void}
 */
void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid)
{
    xSemaphoreTake(chassis_pid->xMutex_pid, (TickType_t)10);
    chassis_pid->Pid_pos_x.feedback = feedback_x;
    chassis_pid->Pid_pos_y.feedback = feedback_y;
    chassis_pid->Pid_pos_w.feedback = feedback_w;
    xSemaphoreGive(chassis_pid->xMutex_pid);
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