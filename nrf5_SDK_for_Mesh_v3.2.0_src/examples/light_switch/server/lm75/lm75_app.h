#ifndef _LM75_APP_H_
#define _LM75_APP_H_

#include <stdio.h>
#include "boards.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"

void temp_sensor_init();
uint8_t get_sensor_temp();

#endif
