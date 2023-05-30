#ifndef __CHASSIS_REMOTE_CONTROL__
#define __CHASSIS_REMOTE_CONTROL__

#include "wtr_mavlink.h"
#include "chassis_commen.h"

void RemoteControlInit();
void RemoteControlStart();
void RemoteControlSendMsg(JOYSTICK_SEND *msg_joystick_send);
void JoystickSwitchLED(float r, float g, float b, float lightness, uint16_t duration, JOYSTICK_SEND *msg_joystick_send);
void JoystickSwitchTitle(uint8_t id, char title[20], JOYSTICK_SEND *msg_joystick_send);
void JoystickSwitchMsg(uint8_t id, char message[20], JOYSTICK_SEND *msg_joystick_send);
void JoystickDelete(uint8_t id, JOYSTICK_SEND *msg_joystick_send);
bool ReadJoystickButtons(mavlink_joystick_air_t msg_joystick_air_, KEYS index);
float ReadJoystickLeft_x(mavlink_joystick_air_t msg_joystick_air_);
float ReadJoystickLeft_y(mavlink_joystick_air_t msg_joystick_air_);
float ReadJoystickRight_x(mavlink_joystick_air_t msg_joystick_air_);
float ReadJoystickRight_y(mavlink_joystick_air_t msg_joystick_air_);
int16_t ReadJoystickKnobsLeft_x(mavlink_joystick_air_t msg_joystick_air_);
int16_t ReadJoystickKnobsLeft_y(mavlink_joystick_air_t msg_joystick_air_);
bool ReadJoystickSwitchs(mavlink_joystick_air_t msg_joystick_air_, SWITCHS index);

extern mavlink_joystick_air_t msg_joystick_air;
extern JOYSTICK_SEND msg_joystick_send;

#endif