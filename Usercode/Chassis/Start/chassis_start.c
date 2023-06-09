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
#include "chassis_start.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "chassis_perception.h"
#include "chassis_communicate.h"
#include "chassis_commen.h"
#include "chassis_remote_control.h"
#include "chassis_operate_app.h"


// mavlink_controller_t ControllerData = {0};

/**
 * @description: 初始化与开启线程
 * @author: szf
 * @date:
 * @return {void}
 */
void StartDefaultTask(void const *argument)
{
    /*初始化*/
    CommunicateInit(); // 通信初始化
    ChassisInit();     // 底盘组件初始化
    MotorInit();       // 电机初始化
    PerceptionInit();  // 定位组件初始化
    RemoteControlInit();
    vTaskDelay(100);

    /*开启线程*/
    taskENTER_CRITICAL();
    ChassisStateMachineTaskStart(); // 底盘状态机
    CommunicateTaskStart();         // 通信线程
    ServoTaskStart();               // 伺服线程
    StateManagemantTaskStart();     // 状态切换线程
    // PerceptionTaskStart();          // 底盘感知定位线程
    RemoteControlStart();
    taskEXIT_CRITICAL();
    
    for (;;) {
        vTaskDelay(2);
    }
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