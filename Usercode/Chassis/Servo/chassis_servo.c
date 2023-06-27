/**
 * @author OriTwil
 * @brief 伺服控制底盘
 * @version 1.0
 * @date 2023-5-18
 *
 */

#include "chassis_servo.h"
#include "chassis_config.h"
#include "chassis_commen.h"

CHASSIS_PID Chassis_Pid;

/**
 * @description: 伺服线程
 * @author: szf
 * @return {void}
 */
void ServoTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        // 更新PID的目标值
        xSemaphoreTake(Chassis_Control.xMutex_control, (TickType_t)10);
        SetPIDTarget(Chassis_Control.Chassis_Control_x,
                     Chassis_Control.Chassis_Control_y,
                     Chassis_Control.Chassis_Control_w,
                     &Chassis_Pid);
        xSemaphoreGive(Chassis_Control.xMutex_control);
        // 更新PID的反馈值
        xSemaphoreTake(Chassis_Position.xMutex_position, (TickType_t)10);
        SetPIDFeedback(Chassis_Position.Chassis_Position_x,
                       Chassis_Position.Chassis_Position_y,
                       Chassis_Position.Chassis_Position_w,
                       &Chassis_Pid);
        xSemaphoreGive(Chassis_Position.xMutex_position);
        // 麦轮解算
        xSemaphoreTake(Chassis_Control.xMutex_control, (TickType_t)10);
        CalculateFourMecanumWheels(moter_speed,
                                   Chassis_Control.Chassis_Control_vx,
                                   Chassis_Control.Chassis_Control_vy,
                                   Chassis_Control.Chassis_Control_vw);
        xSemaphoreGive(Chassis_Control.xMutex_control);

        // 伺服控制
        positionServo(Baffle.position_servo_ref_baffle, &hDJI[7]);
        speedServo(moter_speed[0], &hDJI[0]);
        speedServo(moter_speed[1], &hDJI[1]);
        speedServo(moter_speed[2], &hDJI[2]);
        speedServo(moter_speed[3], &hDJI[3]);
        CanTransmit_DJI_1234(&hcan1, hDJI[0].speedPID.output,
                             hDJI[1].speedPID.output,
                             hDJI[2].speedPID.output,
                             hDJI[3].speedPID.output);
        CanTransmit_DJI_5678(&hcan1,
                             hDJI[4].speedPID.output,
                             hDJI[5].speedPID.output,
                             hDJI[6].speedPID.output,
                             hDJI[7].speedPID.output);
        vTaskDelayUntil(&PreviousWakeTime, 2);
    }
}

/**
 * @description: 伺服测试线程，比赛时关掉
 * @author: szf
 * @return {void}
 */
void ServoTestTask(void const *argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();
    vTaskDelay(20);
    for (;;) {
        vTaskDelayUntil(&PreviousWakeTime, 3);
    }
}

/**
 * @description: 创建伺服线程
 * @author: szf
 * @return {void}
 */
void ServoTaskStart()
{
    osThreadDef(servo, ServoTask, osPriorityBelowNormal, 0, 512);
    osThreadCreate(osThread(servo), NULL);

    // osThreadDef(servo_test,ServoTestTask,osPriorityBelowNormal,0,512);
    // osThreadCreate(osThread(servo_test),NULL);
}

// 电机初始化
void MotorInit()
{
    CANFilterInit(&hcan1);
    hDJI[0].motorType = M3508; // 右前
    hDJI[1].motorType = M3508; // 左前
    hDJI[2].motorType = M3508; // 左后
    hDJI[3].motorType = M3508; // 右后
    hDJI[4].motorType = M3508; // 挡板
    hDJI[5].motorType = M3508;
    hDJI[6].motorType = M3508;
    hDJI[7].motorType = M3508;
    DJI_Init(); // 大疆电机初始化
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

CHASSIS_PID ReadChassisPID(CHASSIS_PID *chassis_pid)
{
    CHASSIS_PID chassis_pid_temp;
    xSemaphoreTake(chassis_pid->xMutex_pid, (TickType_t)10);
    chassis_pid_temp = *chassis_pid;
    xSemaphoreGive(chassis_pid->xMutex_pid);
    return chassis_pid_temp;
}