#ifndef __CHASSIS_REMOTE_CONTROL__
#define __CHASSIS_REMOTE_CONTROL__

#include "wtr_mavlink.h"
#include "chassis_commen.h"

void RemoteControlInit();
void RemoteControlStart();
void RemoteControlSendMsg(JOYSTICK_AIR_DASHBOARD_SET_MSG *Msg_joystick_air_msg);
void JoystickSwitchLED(float r, float g, float b, float lightness, uint16_t duration, JOYSTICK_AIR_LED *Msg_joystick_air_led);
void JoystickSwitchTitle(uint8_t id, char title[20], JOYSTICK_AIR_DASHBOARD_SET_TITLE *Msg_joystick_air_title);
void JoystickSwitchMsg(uint8_t id, char message[20], JOYSTICK_AIR_DASHBOARD_SET_MSG *Msg_joystick_air_msg);
void JoystickDelete(uint8_t id, JOYSTICK_AIR_DASHBOARD_DELETE *Msg_joystick_air_delete);
bool ReadJoystickButtons(JOYSTICK_AIR *msg_joystick_air_, KEYS index);
float ReadJoystickLeft_x(JOYSTICK_AIR *msg_joystick_air_);
float ReadJoystickLeft_y(JOYSTICK_AIR *msg_joystick_air_);
float ReadJoystickRight_x(JOYSTICK_AIR *msg_joystick_air_);
float ReadJoystickRight_y(JOYSTICK_AIR *msg_joystick_air_);
int16_t ReadJoystickKnobsLeft(JOYSTICK_AIR *msg_joystick_air_);
int16_t ReadJoystickKnobsRight(JOYSTICK_AIR *msg_joystick_air_);
bool ReadJoystickSwitchs(JOYSTICK_AIR *msg_joystick_air_, SWITCHS index);

void TitleInit();
void MsgUpdatePoint();
void MsgUpdateState();
void MsgUpdatePosture();
void MsgUpdateKnobR();
void LedUpdate();

extern JOYSTICK_AIR Msg_joystick_air;
extern JOYSTICK_AIR_LED msg_joystick_air_led;
extern JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_point;
extern JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_state;
extern JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_posture;
extern JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_knob_r;
extern JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_point;
extern JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_state;
extern JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_posture;
extern JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_knob_r;
extern JOYSTICK_AIR_DASHBOARD_DELETE msg_joystick_air_delete;

#endif