#ifndef __CHASSIS_SERVO_H__
#define __CHASSIS_SERVO_H__

#include "chassis_state_machine.h"
#include "math.h"
#include "chassis_config.h"
#include "chassis_commen.h"

void ServoTaskStart();
void MotorInit();

void SetPIDTarget(float target_x, float target_y, float target_w, CHASSIS_PID *chassis_pid);

void SetPIDFeedback(float feedback_x, float feedback_y, float feedback_w, CHASSIS_PID *chassis_pid);

CHASSIS_PID ReadChassisPID(CHASSIS_PID *chassis_pid);

extern CHASSIS_PID Chassis_Pid;

#endif