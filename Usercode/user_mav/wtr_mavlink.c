/*
 * @Author: szf01 2176529058@qq.com
 * @Date: 2022-10-23 13:50:44
 * @LastEditors: szf01 2176529058@qq.com
 * @LastEditTime: 2022-12-04 12:52:46
 * @FilePath: /underpan/usercode/user_mav/wtr_mavlink.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file wtr_mavlink.c
 * @author X. Y.
 * @brief 移植到 stm32 的 mavlink（只需要包含 wtr_mavlink.h 就够了）
 * @version 1.1
 * @date 2022-09-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "wtr_mavlink.h"

wtrMavlink_handle_t hMAVLink[MAVLINK_COMM_NUM_BUFFERS] = {0};

/**
 * @description: 
 * @param {UART_HandleTypeDef} *huart
 * @param {mavlink_channel_t} chan
 * @return {*}
 */
void wtrMavlink_BindChannel(UART_HandleTypeDef *huart, mavlink_channel_t chan)
{
    hMAVLink[chan].huart = huart;
}

int wtrMavlink_StartReceiveIT(mavlink_channel_t chan)
{
    if (hMAVLink[chan].huart == NULL) {
        return -1;
    }

    return WTR_UART_Receive_IT(hMAVLink[chan].huart, &(hMAVLink[chan].rx_buffer), 1);
}

__weak void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg)
{
    switch (msg->msgid) {
        case 1:
            // id = 1 的消息对应的解码函数(mavlink_msg_xxx_decode)
            break;
        case 2:
            // id = 2 的消息对应的解码函数(mavlink_msg_xxx_decode)
            break;
        // ......
        default:
            break;
    }
}
