/**
 * @OriTwil
 * @Date: 2023-02-22 10:59:01
 * @Description: 主函数
 * @WeChat:szf13373959031
 **/

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_dji.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "user_main.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "user_callback.h"
#include "chassis_perception.h"

// mavlink_controller_t ControllerData = {0};

/**
 * @description: 初始化与开启线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    ChassisInit();
    MotorInit();                                            // 电机初始化
    wtrMavlink_BindChannel(&huart_Computer, MAVLINK_COMM_0);        // MAVLINK初始化
    CtrlDataSender_Init(&huart_Remote_Control, MAVLINK_COMM_1);           // 遥控器初始化
    HAL_UART_Receive_DMA(&huart_AS69, JoyStickReceiveData, 18); // DMA接收AS69
    vTaskDelay(100);

    // 开启线程
    // ChassisStateMachineTaskStart();       // 全向轮底盘控制线程
    PerceptionTaskStart();      // 底盘感知定位线程
    CommunicateTaskStart();     // 通信线程
    StateManagemantTaskStart(); // 状态切换线程
    ServoTaskStart();

    for (;;) {
        vTaskDelay(1);
    }
}

void CtrlDataSender_Init(UART_HandleTypeDef *huart, mavlink_channel_t chan)
{
    // WTR_MAVLink_Init(huart, chan);
    wtrMavlink_BindChannel(huart, MAVLINK_COMM_1); // MAVLINK初始化
    CtrlDataSendChan = chan;
}

/**
 * @description: 闪A板上的灯(测试用)
 * @date:
 * @return {void}
 */
void BlinkLED()
{
    static int n_ = 0;
    if (n_++ > 100) {
        n_ = 0;
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
    }
}