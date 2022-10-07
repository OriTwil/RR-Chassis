#ifndef USERMAIN_H
#define USERMAIN_H

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

#define pi 3.1415926535898
#define DEC (pi/180)
#define r_underpan 0.1934
#define r_wheel 0.076

void calculate(double * moter_speed,
               double v_x,
               double v_y,
               double v_w);

extern double moter_speed [3];
#endif