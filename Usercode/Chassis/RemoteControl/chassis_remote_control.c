#include "chassis_remote_control.h"
#include "chassis_commen.h"
#include "chassis_config.h"
#include "wtr_mavlink.h"
#include "wtr_uart.h"
#include "chassis_communicate.h"
#include "chassis_state_machine.h"

JOYSTICK_AIR Msg_joystick_air;
JOYSTICK_AIR_LED msg_joystick_air_led;
JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_point;
JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_state;
JOYSTICK_AIR_DASHBOARD_SET_TITLE msg_joystick_air_title_posture;
JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_point;
JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_state;
JOYSTICK_AIR_DASHBOARD_SET_MSG msg_joystick_air_msg_posture;
JOYSTICK_AIR_DASHBOARD_DELETE msg_joystick_air_delete;

char title_point[20]   = "point";
char title_state[20]   = "state";
char title_posture[20] = "posture";
char msg_point[20]     = "no_msg";
char msg_state[20]     = "no_msg";
char msg_posture[20]   = "no_msg";
#define ID_Point   6
#define ID_State   8
#define ID_Posture 16

void RemoteControlTask(void const *argument)
{
    uint32_t PreviousWakeTime = xTaskGetTickCount();
    uint32_t counter          = 0;

    while (1) {
        if (counter % 50 == 0) {
            TitleInit();
            counter = 0;
        }
        counter++;

        // 底盘位置
        MsgUpdatePoint();
        // 底盘状态
        MsgUpdateState();
        // 位置
        MsgUpdatePosture();

        RemoteControlSendMsg(&msg_joystick_air_msg_point);
        RemoteControlSendMsg(&msg_joystick_air_msg_state);
        RemoteControlSendMsg(&msg_joystick_air_msg_posture);
        vTaskDelayUntil(&PreviousWakeTime, 100);
    }
}

void RemoteControlInit()
{
    wtrMavlink_BindChannel(&huart_Remote_Control, MAVLINK_COMM_1); // MAVLINK遥控器
    wtrMavlink_StartReceiveIT(MAVLINK_COMM_1);                     // 以mavlink接收遥控器

    HAL_UART_Receive_DMA(&huart1, JoyStickReceiveData, 18); // DMA接收AS69
}

void RemoteControlStart()
{
    osThreadDef(RemoteControl, RemoteControlTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(RemoteControl), NULL);
}

void RemoteControlSendMsg(JOYSTICK_AIR_DASHBOARD_SET_MSG *Msg_joystick_air_msg)
{
    xSemaphoreTake(Msg_joystick_air_msg->xMutex_joystick_air_dashboard_set_msg, portMAX_DELAY);
    mavlink_joystick_air_dashboard_set_msg_t msg_joystick_air_dashboard_set_msg_temp = Msg_joystick_air_msg->msg_joystick_air_dashboard_set_msg;
    xSemaphoreGive(Msg_joystick_air_msg->xMutex_joystick_air_dashboard_set_msg);

    mavlink_msg_joystick_air_dashboard_set_msg_send_struct(MAVLINK_COMM_1, &msg_joystick_air_dashboard_set_msg_temp);
    vTaskDelay(25);
}

void JoystickSwitchLED(float r, float g, float b, float lightness, uint16_t duration, JOYSTICK_AIR_LED *Msg_joystick_air_led)
{
    xSemaphoreTake(Msg_joystick_air_led->xMutex_joystick_air_led, portMAX_DELAY);
    Msg_joystick_air_led->msg_joystick_air_led.r         = r;
    Msg_joystick_air_led->msg_joystick_air_led.g         = g;
    Msg_joystick_air_led->msg_joystick_air_led.b         = b;
    Msg_joystick_air_led->msg_joystick_air_led.lightness = lightness;
    Msg_joystick_air_led->msg_joystick_air_led.duration  = duration;
    JOYSTICK_AIR_LED msg_joystick_send_temp              = *Msg_joystick_air_led;
    xSemaphoreGive(Msg_joystick_air_led->xMutex_joystick_air_led);

    mavlink_msg_joystick_air_led_send_struct(MAVLINK_COMM_1, &msg_joystick_send_temp.msg_joystick_air_led);
}

void JoystickSwitchTitle(uint8_t id, char title[20], JOYSTICK_AIR_DASHBOARD_SET_TITLE *Msg_joystick_air_title)
{
    xSemaphoreTake(Msg_joystick_air_title->xMutex_joystick_air_dashboard_set_title, portMAX_DELAY);
    Msg_joystick_air_title->msg_joystick_air_dashboard_set_title.id = id;
    strncpy(Msg_joystick_air_title->msg_joystick_air_dashboard_set_title.title, title, 20);
    JOYSTICK_AIR_DASHBOARD_SET_TITLE Msg_joystick_air_title_temp = *Msg_joystick_air_title;
    xSemaphoreGive(Msg_joystick_air_title->xMutex_joystick_air_dashboard_set_title);

    mavlink_msg_joystick_air_dashboard_set_title_send_struct(MAVLINK_COMM_1, &Msg_joystick_air_title_temp.msg_joystick_air_dashboard_set_title);
    vTaskDelay(26);
}

void JoystickSwitchMsg(uint8_t id, char message[20], JOYSTICK_AIR_DASHBOARD_SET_MSG *Msg_joystick_air_msg)
{
    xSemaphoreTake(Msg_joystick_air_msg->xMutex_joystick_air_dashboard_set_msg, portMAX_DELAY);
    Msg_joystick_air_msg->msg_joystick_air_dashboard_set_msg.id = id;
    strncpy(Msg_joystick_air_msg->msg_joystick_air_dashboard_set_msg.message, message, 20);
    xSemaphoreGive(Msg_joystick_air_msg->xMutex_joystick_air_dashboard_set_msg);
}

void JoystickDelete(uint8_t id, JOYSTICK_AIR_DASHBOARD_DELETE *Msg_joystick_air_delete)
{
    xSemaphoreTake(Msg_joystick_air_delete->xMutex_joystick_air_dashboard_del, portMAX_DELAY);
    Msg_joystick_air_delete->msg_joystick_air_dashboard_del.id = id;
    JOYSTICK_AIR_DASHBOARD_DELETE Msg_joystick_air_delete_temp = *Msg_joystick_air_delete;
    xSemaphoreGive(Msg_joystick_air_delete->xMutex_joystick_air_dashboard_del);

    mavlink_msg_joystick_air_dashboard_del_send_struct(MAVLINK_COMM_1, &Msg_joystick_air_delete_temp.msg_joystick_air_dashboard_del);
}

bool ReadJoystickButtons(JOYSTICK_AIR *msg_joystick_air_, KEYS index)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);

    return ((msg_joystick_air_temp.buttons >> (index - 1)) & 1);
}

float ReadJoystickLeft_x(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);
    return msg_joystick_air_temp.joystickL[0];
}

float ReadJoystickLeft_y(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);
    return msg_joystick_air_temp.joystickL[1];
}

float ReadJoystickRight_x(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);
    return msg_joystick_air_temp.joystickR[0];
}

float ReadJoystickRight_y(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);
    return msg_joystick_air_temp.joystickR[1];
}

int16_t ReadJoystickKnobsLeft_x(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);

    return msg_joystick_air_temp.knobs[0];
}

int16_t ReadJoystickKnobsLeft_y(JOYSTICK_AIR *msg_joystick_air_)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);

    return msg_joystick_air_temp.knobs[1];
}

bool ReadJoystickSwitchs(JOYSTICK_AIR *msg_joystick_air_, SWITCHS index)
{
    xSemaphoreTake(msg_joystick_air_->xMutex_joystick_air, portMAX_DELAY);
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_->msg_joystick_air;
    vPortExitCritical();
    xSemaphoreGive(msg_joystick_air_->xMutex_joystick_air);

    return ((msg_joystick_air_temp.switchs >> index) & 1);
}

// 应用函数
void TitleInit()
{
    JoystickSwitchTitle(ID_Point, title_point, &msg_joystick_air_title_point);
    JoystickSwitchTitle(ID_State, title_state, &msg_joystick_air_title_state);
    JoystickSwitchTitle(ID_Posture, title_posture, &msg_joystick_air_title_posture);
}

void MsgUpdatePoint()
{
    vPortEnterCritical();
    int point_temp = mav_posture.point;
    vPortExitCritical();
    snprintf(msg_point, sizeof(msg_point), "%d", point_temp);
    JoystickSwitchMsg(ID_Point, msg_point, &msg_joystick_air_msg_point);
    vTaskDelay(2);
}

void MsgUpdatePosture()
{
    vPortEnterCritical();
    float posture_x = mav_posture.pos_x;
    float posture_y = mav_posture.pos_y;
    float posture_w = mav_posture.zangle;
    vPortExitCritical();

    snprintf(msg_posture, sizeof(msg_posture), "x=%d y=%d w=%d", (int32_t)(posture_x * 10000), (int32_t)(posture_y * 10000), (int32_t)(posture_w * 10000));
    JoystickSwitchMsg(ID_Posture, msg_posture, &msg_joystick_air_msg_posture);
    vTaskDelay(2);
}

void MsgUpdateState()
{
    xSemaphoreTake(Robot_state.xMutex_Robot, portMAX_DELAY);
    ROBOT_STATE Robot_state_temp = Robot_state;
    xSemaphoreGive(Robot_state.xMutex_Robot);
    switch (Robot_state_temp.Chassis_state) {
        case Locked:
            snprintf(msg_state, sizeof(msg_state), "Locked");
            break;
        case RemoteControl:

            snprintf(msg_state, sizeof(msg_state), "RemoteControl");
            break;
        case AutoControl:

            snprintf(msg_state, sizeof(msg_state), "AutoControl");
            break;
        default:
            snprintf(msg_state, sizeof(msg_state), "ERROR");
            break;
    }
    JoystickSwitchMsg(ID_State, msg_state, &msg_joystick_air_msg_state);
    vTaskDelay(2);
}