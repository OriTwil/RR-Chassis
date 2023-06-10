/**
 * @description: 回调函数
 * @param {UART_HandleTypeDef} *huart
 * @author: szf
 * @date:
 * @return {void}
 */

#include "chassis_start.h"
#include "wtr_callback.h"
#include "wtr_uart.h"
#include "wtr_mavlink.h"
#include "chassis_communicate.h"
#include "chassis_remote_control.h"

int16_t crldata[4] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 上位机消息
    if (huart->Instance == UART_Computer) {
        wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_0); // 进入mavlink回调
    }
    // 定位模块消息
    if (huart->Instance == UART_OPS) // 底盘定位系统的decode
    {
        OPS_Decode();
        // mavlink_msg_posture_send_struct(MAVLINK_COMM_0, &mav_posture);
    }
    if (huart->Instance == UART_AS69) {
        AS69_Decode(); // AS69解码
    }
    if (huart->Instance == UART_Remote_Control) {
        wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_1); // 进入mavlink回调
    }
}

/**
 * @brief 接收到完整消息且校验通过后会调用这个函数。在这个函数里调用解码函数就可以向结构体写入收到的数据
 *
 * @param msg 接收到的消息
 * @return
 */
// extern mavlink_controller_t ControllerData;
void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg)
{

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_CONTROL:
            // id = 9 的消息对应的解码函数(mavlink_msg_xxx_decode)
            mavlink_msg_control_decode(msg, &control);
            break;
        case MAVLINK_MSG_ID_JOYSTICK_AIR:
            mavlink_msg_joystick_air_decode(msg, &Msg_joystick_air.msg_joystick_air);
            break;
        // ......
        default:
            break;
    }
}

int test_count = 0;
/**
 * @description:外部中断回调函数
 * @author: szf
 * @date:
 * @return {void}
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_12:
            break;
        case GPIO_PIN_15:
            break;
        case GPIO_PIN_13:
            break;
        default:
            break;
    }
}
