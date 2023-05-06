/**
 * @description: 回调函数
 * @param {UART_HandleTypeDef} *huart
 * @author: szf
 * @date:
 * @return {void}
 */

#include "usermain.h"
#include "usercallback.h"
#include "wtr_uart.h"
#include "uart_device.h"
#include "wtr_mavlink.h"

int counter          = 0;
int test             = 0;
float w_speed        = 0;
int16_t crldata[4] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    test++;
    // 上位机消息
    if (huart->Instance == USART2) {
        // UART1Decode();//AS69解码
        wtrMavlink_UARTRxCpltCallback(huart, MAVLINK_COMM_0); // 进入mavlink回调
    }
    // 定位模块消息
    if (huart->Instance == UART4) // 底盘定位系统的decode,可以换为DMA轮询,封装到祖传的串口库里s
    {
        OPS_Decode();
    }
    else {
        AS69_Decode(); // AS69解码
        // crl_speed.vy = (float ) (crldata[0] - CH0_BIAS)/CH_RANGE * 1;
        // crl_speed.vx = (float ) (crldata[1] - CH1_BIAS)/CH_RANGE * 1;
        // crl_speed.vw = (float ) (crldata[2] - CH2_BIAS)/CH_RANGE * 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UD_TxCpltCallback(huart);
}

/**
 * @brief 接收到完整消息且校验通过后会调用这个函数。在这个函数里调用解码函数就可以向结构体写入收到的数据
 *
 * @param msg 接收到的消息
 * @return
 */
extern mavlink_controller_t ControllerData;
void wtrMavlink_MsgRxCpltCallback(mavlink_message_t *msg)
{

    switch (msg->msgid) {
        case 9:
            // id = 9 的消息对应的解码函数(mavlink_msg_xxx_decode)
            mavlink_msg_control_set_decode(msg, &control);
            break;
        case 1:
            // id = 1 的消息对应的解码函数(mavlink_msg_xxx_decode)
            mavlink_msg_controller_decode(msg, &ControllerData); // 遥控器 
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
extern uni_wheel_t wheels[3];
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    test_count++;
    switch (GPIO_Pin)
	{
	case GPIO_PIN_12:
		Wheel_Hall_Callback(GPIOE, GPIO_Pin, &wheels[0]);
		break;
	case GPIO_PIN_15:
		Wheel_Hall_Callback(GPIOE, GPIO_Pin, &wheels[1]);
		// UWheels_Hall_Callback(1);
		break;
	case GPIO_PIN_13:
		Wheel_Hall_Callback(GPIOE, GPIO_Pin, &wheels[2]);
        test_count++;
		// UWheels_Hall_Callback(2);
		break;
	default:
        counter++;
		// printf("EXTI %d\n", GPIO_Pin);
		break;
	}
}
