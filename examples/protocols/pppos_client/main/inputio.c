#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "inputio.h"
#include "unistd.h"
#include "leds.h"
#include "global_variable.h"


static const char *TAG = "74hc165";
//static unsigned short io_value = 0x0000;
static SemaphoreHandle_t semp_io_read;

unsigned short IO74HC165ReadData(void)
{
    unsigned char i;
    unsigned short temp = 0;

    gpio_set_level(18, 0);
    usleep(1);
    gpio_set_level(18, 1);
    for(i=0;i<16;i++)
    {
        usleep(1);
        /*gpio_set_level(19, 0);
        usleep(1);
        gpio_set_level(19, 1);*/
        if(gpio_get_level(21))
        {
            temp |= (1 << (15 - i));
        }

        gpio_set_level(19, 0);
        usleep(1);
        gpio_set_level(19, 1);
    }
    return temp;
}

void inputio_init(void)
{
    /* data pin */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<21);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //gpio_set_level(21, 0);

    /* sck pin */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<19);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(19, 1);

    /* rck pin */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<18);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(18, 1);

    semp_io_read = xSemaphoreCreateMutex();
    if( semp_io_read == NULL )
    {
        ESP_LOGE(TAG, "Input IO semphr create error");
    }
    xSemaphoreGive(semp_io_read);
}

unsigned short input_pin_read(void)
{
    static unsigned short io_value_store = 0xFFFF;
    unsigned short io_value;
    xSemaphoreTake(semp_io_read, portMAX_DELAY);
    io_value = IO74HC165ReadData();
    xSemaphoreGive(semp_io_read);
    if(io_value != io_value_store)
    {
        io_value_store = io_value;
        xEventGroupSetBits(event_group, DATA_REPORT_BIT);
    }
    //ESP_LOGI(TAG, "input_io = 0x%04X", io_value);

    return io_value;
}

void input_io_task(void *pvParameter)
{
    ESP_LOGI(TAG, "%s started", __func__);
    while(1)
    {
        vTaskDelay(50 / portTICK_RATE_MS);
        ESP_LOGD(TAG, "Read inputio = 0x%04X", input_pin_read());
        led_on(input_pin_read() & 0x0FFF, true);
        led_on((~input_pin_read()) & 0x0FFF, false);
    }
}