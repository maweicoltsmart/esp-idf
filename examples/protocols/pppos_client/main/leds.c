#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "leds.h"
#include "unistd.h"
#include "inputio.h"

static const char *TAG = "hc595_led";
static SemaphoreHandle_t semp_led;
static unsigned short led_state = 0;

esp_timer_handle_t rs485tx_timer, rs485rx_timer; // oneshot_timer
esp_timer_handle_t data_report_timer, work_online_timer, work_offline_timer, work_upgrade_timer; // periodic_timer


void HC595SendData(unsigned short SendVal)
{
    unsigned char i;
    for(i=0;i<16;i++)
    {
        if((SendVal<<i)&0x8000)
        {
            gpio_set_level(14, 1);
        }
        else
        {
            gpio_set_level(14, 0);
        }
        gpio_set_level(26, 0);
        usleep(1);
        gpio_set_level(26, 1);
    }
    gpio_set_level(27, 0);
    usleep(1);
    gpio_set_level(27, 1);
}

void HC595_gpio_init(void)
{
    /* data pin */
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<14);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(14, 0);

    /* sck pin */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<26);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(26, 0);

    /* rck pin */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<27);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(27, 0);
}

void led_on(unsigned short led, bool on)
{
    xSemaphoreTake(semp_led, portMAX_DELAY);
    if(on)
    {
        led_state |= led;
    }
    else
    {
        led_state &= ~led;
    }
    HC595SendData(led_state);
    xSemaphoreGive(semp_led);
}

static void rs485tx_timer_callback(void* arg)
{
    led_on(HC595_LED12, false);
}

void rs485tx_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &rs485tx_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "rs485tx"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &rs485tx_timer));
}

void rs485tx_led_blink(void)
{
    led_on(HC595_LED12, true);
    ESP_ERROR_CHECK(esp_timer_start_once(rs485tx_timer, 150000));
}

static void rs485rx_timer_callback(void* arg)
{
    led_on(HC595_LED13, false);
}

void rs485rx_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &rs485rx_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "rs485rx"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &rs485rx_timer));
}

void rs485rx_led_blink(void)
{
    led_on(HC595_LED13, true);
    ESP_ERROR_CHECK(esp_timer_start_once(rs485rx_timer, 150000));
}

static void data_report_timer_callback(void* arg)
{
    led_on(HC595_LED14, false);
}

void data_report_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &data_report_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "data-report"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &data_report_timer));
}

void data_report_led_blink(void)
{
    led_on(HC595_LED14, true);
    ESP_ERROR_CHECK(esp_timer_start_once(data_report_timer, 500000));
}

static void work_offline_timer_callback(void* arg)
{
    static bool ledon = false;
    led_on(HC595_LED15, ledon);
    ledon = !ledon;
}

void work_offline_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &work_offline_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "work-offline"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &work_offline_timer));
}

void work_offline_led_stop(void)
{
    //ESP_ERROR_CHECK(esp_timer_stop(work_offline_timer));
    esp_timer_stop(work_offline_timer);
}

void work_offline_led_blink(void)
{
    led_on(HC595_LED15, true);
    ESP_ERROR_CHECK(esp_timer_start_periodic(work_offline_timer, 200000));
}

static void work_online_timer_callback(void* arg)
{
    static bool ledon = false;
    led_on(HC595_LED15, ledon);
    ledon = !ledon;
}

void work_online_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &work_online_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "work-online"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &work_online_timer));
}

void work_online_led_stop(void)
{
    //ESP_ERROR_CHECK(esp_timer_stop(work_online_timer));
    esp_timer_stop(work_online_timer);
}

void work_online_led_blink(void)
{
    led_on(HC595_LED15, true);
    ESP_ERROR_CHECK(esp_timer_start_periodic(work_online_timer, 1000000));
}

static void work_upgrade_timer_callback(void* arg)
{
    static bool ledon = false;
    led_on(HC595_LED15, ledon);
    ledon = !ledon;
}

void work_upgrade_led_init(void)
{
    const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &work_upgrade_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .name = "work-upgrade"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &work_upgrade_timer));
}

void work_upgrade_led_stop(void)
{
    //ESP_ERROR_CHECK(esp_timer_stop(work_upgrade_timer));
    esp_timer_stop(work_upgrade_timer);
}

void work_upgrade_led_blink(void)
{
    led_on(HC595_LED15, true);
    ESP_ERROR_CHECK(esp_timer_start_periodic(work_upgrade_timer, 100000));
}

void led_poweron(void)
{
    ESP_LOGI(TAG, "LED power on init");
    HC595_gpio_init();
    semp_led = xSemaphoreCreateMutex();
    if( semp_led == NULL )
    {
        ESP_LOGE(TAG, "LED semphr create error");
    }
    xSemaphoreGive(semp_led);
    ESP_LOGI(TAG, "LED all off");
    led_on(HC595_LED_ALL, true);
    vTaskDelay(500 / portTICK_RATE_MS);
    led_on(HC595_LED_ALL, false);
    rs485tx_led_init();
    rs485rx_led_init();
    data_report_led_init();
    work_online_led_init();
    work_offline_led_init();
    work_upgrade_led_init();
    /*led_on(HC595_LED11, true);
    while(1)
    {
        vTaskDelay(500 / portTICK_RATE_MS);
        led_on(input_pin_read(), true);
        led_on(~input_pin_read(), false);
    }*/
}
