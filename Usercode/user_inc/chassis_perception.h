#ifndef CHASSIS_PERCEPTION_H
#define CHASSIS_PERCEPTION_H

#include "user_main.h"
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
#include "chassis_state_management.h"

void PerceptionTaskStart(mavlink_controller_t *ctrl_data);

#endif