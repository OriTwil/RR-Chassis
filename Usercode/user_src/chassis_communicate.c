/**
 * @file controller_data_sender.c
 * @author TITH (1023515576@qq.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "chassis_communicate.h"
#include "user_main.h"

void CommunicateTask(void const *argument)
{
	while (1)
	{
		mavlink_msg_controller_send_struct(CtrlDataSendChan, argument);
		mavlink_msg_posture_send_struct(MAVLINK_COMM_0,&mav_posture);
		osDelay(10);
	}
}

void CommunicateTaskStart(mavlink_controller_t* controller)
{
	osThreadDef(communicate, CommunicateTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(communicate), controller);
}
