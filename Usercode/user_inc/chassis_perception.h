#ifndef CHASSIS_PERCEPTION_H
#define CHASSIS_PERCEPTION_H

#include "usermain.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "DJI.h"
#include "wtr_uart.h"
#include <math.h>
#include "main.h"
#include "usermain.h"
#include "wtr_mavlink.h"
#include <math.h>
#include "usercallback.h"
#include "usercalculate.h"

void PerceptionTaskStart(mavlink_controller_t *ctrl_data);

#endif