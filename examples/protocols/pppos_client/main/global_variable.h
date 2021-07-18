#ifndef __GLOBAL_VARIABLE_H__
#define __GLOBAL_VARIABLE_H__
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

EventGroupHandle_t event_group;
const int CONNECT_BIT;
const int STOP_BIT;
const int GOT_DATA_BIT;
const int DATA_REPORT_BIT;

#endif