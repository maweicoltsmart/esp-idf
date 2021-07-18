#ifndef __INPUT_IO_H__
#define __INPUT_IO_H__
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

void inputio_init(void);
unsigned short input_pin_read(void);
void input_io_task(void *pvParameter);

#endif