#ifndef CHASSIS_PERCEPTION_H
#define CHASSIS_PERCEPTION_H

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "wtr_can.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "usart.h"
#include "chassis_commen.h"

void PerceptionTaskStart();
void PerceptionInit();

void PerceptionSwitchState(PERCEPTION_STATE target_perception_state, ROBOT_STATE *current_robot_state);

#endif