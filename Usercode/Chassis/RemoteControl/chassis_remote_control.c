#include "chassis_remote_control.h"
#include "chassis_commen.h"
#include "chassis_config.h"
#include "wtr_mavlink.h"
#include "wtr_uart.h"

mavlink_joystick_air_t msg_joystick_air;
JOYSTICK_SEND msg_joystick_send;

void RemoteControlTask(void const *argument)
{
    uint32_t PreviousWakeTime = xTaskGetTickCount();
    while (1) {
        RemoteControlSendMsg(&msg_joystick_send);
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

void RemoteControlSendMsg(JOYSTICK_SEND *msg_joystick_send)
{
    xSemaphoreTake(msg_joystick_send->xMutex_joystick, portMAX_DELAY);
    JOYSTICK_SEND msg_joystick_send_temp = *msg_joystick_send;
    xSemaphoreGive(msg_joystick_send->xMutex_joystick);

    mavlink_msg_joystick_air_dashboard_set_msg_send_struct(MAVLINK_COMM_1, &msg_joystick_send_temp.msg_joystick_air_dashboard_set_msg);
}

void JoystickSwitchLED(float r, float g, float b, float lightness, uint16_t duration, JOYSTICK_SEND *msg_joystick_send)
{
    xSemaphoreTake(msg_joystick_send->xMutex_joystick, portMAX_DELAY);
    msg_joystick_send->msg_joystick_air_led.r         = r;
    msg_joystick_send->msg_joystick_air_led.g         = g;
    msg_joystick_send->msg_joystick_air_led.b         = b;
    msg_joystick_send->msg_joystick_air_led.lightness = lightness;
    msg_joystick_send->msg_joystick_air_led.duration  = duration;
    JOYSTICK_SEND msg_joystick_send_temp              = *msg_joystick_send;
    xSemaphoreGive(msg_joystick_send->xMutex_joystick);

    mavlink_msg_joystick_air_led_send_struct(MAVLINK_COMM_1, &msg_joystick_send_temp.msg_joystick_air_led);
}

void JoystickSwitchTitle(uint8_t id, char title[20], JOYSTICK_SEND *msg_joystick_send)
{
    xSemaphoreTake(msg_joystick_send->xMutex_joystick, portMAX_DELAY);
    msg_joystick_send->msg_joystick_air_dashboard_set_title.id = id;
    strncpy(msg_joystick_send->msg_joystick_air_dashboard_set_title.title, title, 20);
    JOYSTICK_SEND msg_joystick_send_temp = *msg_joystick_send;
    xSemaphoreGive(msg_joystick_send->xMutex_joystick);

    mavlink_msg_joystick_air_dashboard_set_title_send_struct(MAVLINK_COMM_1, &msg_joystick_send_temp.msg_joystick_air_dashboard_set_title);
}

void JoystickSwitchMsg(uint8_t id, char message[20], JOYSTICK_SEND *msg_joystick_send)
{
    xSemaphoreTake(msg_joystick_send->xMutex_joystick, portMAX_DELAY);
    msg_joystick_send->msg_joystick_air_dashboard_set_msg.id = id;
    strncpy(msg_joystick_send->msg_joystick_air_dashboard_set_msg.message, message, 20);
    xSemaphoreGive(msg_joystick_send->xMutex_joystick);
}

void JoystickDelete(uint8_t id, JOYSTICK_SEND *msg_joystick_send)
{
    xSemaphoreTake(msg_joystick_send->xMutex_joystick, portMAX_DELAY);
    msg_joystick_send->msg_joystick_air_dashboard_del.id = id;
    JOYSTICK_SEND msg_joystick_send_temp                 = *msg_joystick_send;
    xSemaphoreGive(msg_joystick_send->xMutex_joystick);

    mavlink_msg_joystick_air_dashboard_del_send_struct(MAVLINK_COMM_1, &msg_joystick_send_temp.msg_joystick_air_dashboard_del);
}

bool ReadJoystickButtons(mavlink_joystick_air_t msg_joystick_air_, KEYS index)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();

    return ((msg_joystick_air_temp.buttons >> (index - 1)) & 1);
}

float ReadJoystickLeft_x(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();
    return msg_joystick_air_temp.joystickL[0];
}

float ReadJoystickLeft_y(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();
    return msg_joystick_air_temp.joystickL[1];
}

float ReadJoystickRight_x(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();
    return msg_joystick_air_temp.joystickR[0];
}

float ReadJoystickRight_y(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();
    return msg_joystick_air_temp.joystickR[1];
}

int16_t ReadJoystickKnobsLeft_x(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();

    return msg_joystick_air_temp.knobs[0];
}

int16_t ReadJoystickKnobsLeft_y(mavlink_joystick_air_t msg_joystick_air_)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();

    return msg_joystick_air_temp.knobs[1];
}

bool ReadJoystickSwitchs(mavlink_joystick_air_t msg_joystick_air_, SWITCHS index)
{
    vPortEnterCritical();
    mavlink_joystick_air_t msg_joystick_air_temp = msg_joystick_air_;
    vPortExitCritical();
    return ((msg_joystick_air_temp.switchs >> index) & 1);
}