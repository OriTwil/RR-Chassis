/***
 * @Author: szf
 * @Date: 2023-02-23 19:03:31
 * @LastEditTime: 2023-05-13 19:56:07
 * @LastEditors: szf
 * @Description: 状态机
 * @@WeChat:szf13373959031
 */
#ifndef __CHASSIS_STATE_MACHINE__
#define __CHASSIS_STATE_MACHINE__

#include "can.h"
#include "cmsis_os.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "wtr_calculate.h"
#include "chassis_commen.h"

void ChassisStateMachineTaskStart();

void ChassisSwitchState(CHASSIS_STATE target_chassis_state, ROBOT_STATE *current_robot_state);

void ChassisSwitchPoint(CHASSIS_POINT target_chassis_point, ROBOT_STATE *current_robot_state);

void FireSwitchTarget(FIRE_NUMBER fire_target, FIRE_TARGET *current_fire_target);

void SetChassisPosition(float position_x, float position_y, float position_w, CHASSIS_POSITION *chassis_position);

void SetChassisControlVelocity(float vx_control, float vy_control, float vw_control, CHASSIS_CONTROL *chassis_control);

void SetChassisControlPosition(float x_control, float y_control, float w_control, CHASSIS_CONTROL *chassis_control);

ROBOT_STATE ReadRobotState(ROBOT_STATE *current_robot_state);

CHASSIS_CONTROL ReadChassisControl(CHASSIS_CONTROL *chassis_control);

CHASSIS_POSITION ReadChassisPosition(CHASSIS_POSITION *chassis_position);

mavlink_control_t FrameTransform(mavlink_control_t *control, mavlink_posture_t *posture);

void SpeedSwitchRatio(double target_speed_ratio_linear, double target_speed_ratio_angular, SPEED_RATIO* Speed_Ratio);

void Joystick_Control();

void SetBaffleRef(float target_baffle_ref,BAFFLE *Baffle_);

void SetChassisW(float target_w,CONTROL *Control_);

void SetChassisAdjustmentW(float target_adjustment_w,CONTROL *Control_);

FIRE_NUMBER ReadFireNumber(FIRE_TARGET *current_fire_target);

SPEED_RATIO ReadSpeedRatio(SPEED_RATIO *Speed_ratio_);

void UpdateW();

extern ROBOT_STATE Robot_state;
extern CHASSIS_POSITION Chassis_Position;
extern CHASSIS_CONTROL Chassis_Control;
extern SPEED_RATIO Speed_ratio;
extern BAFFLE Baffle;
extern CONTROL Control;
extern FIRE_TARGET Fire_Target;

#endif