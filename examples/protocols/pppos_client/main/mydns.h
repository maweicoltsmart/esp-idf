#ifndef __MY_DNS_H__
#define __MY_DNS_H__
#include "freertos/FreeRTOS.h"

void get_remote_ip(void);
TickType_t GetCurrentTime(void);
TickType_t TimerGetElapsedTime( TickType_t savedTime );

#endif