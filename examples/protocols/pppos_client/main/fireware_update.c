/* OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "string.h"
#include "param.h"
#include "nvs.h"
#include "nvs_flash.h"

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "leds.h"

QueueHandle_t firmwareupdateversion = NULL;
static const char *TAG = "simple_ota_example";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

#define OTA_URL_SIZE 256

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void simple_ota_example_task(void *pvParameter)
{
    esp_log_level_set("esp_https_ota", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting OTA example");

    esp_http_client_config_t config = {
        .url = NULL,//CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .event_handler = _http_event_handler,
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0) {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif
    firmwareupdateversion = xQueueCreate( 1, sizeof( char* ) );
#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif
    char url_buf[OTA_URL_SIZE];
    char *lValueToSend = NULL;
    portBASE_TYPE xStatus;
upgradeopt:
    ESP_LOGI(TAG, "Wait firmware update command");
    xStatus = xQueueReceive( firmwareupdateversion, &lValueToSend, portMAX_DELAY );
    if( xStatus == pdPASS )
    {
        ESP_LOGI(TAG, "##### Upgrade to %s version", lValueToSend);
    }
    else
    {
        goto upgradeopt;
    }
    work_online_led_stop();
    work_offline_led_stop();
    work_upgrade_led_blink();
    ip_addr_t upgradeserver;
    ESP_LOGI(TAG, "get upgrade server ip ......");
    char *ptr = "upgrade.coltsmart.com";
    ip4_addr_set_u32(&upgradeserver.u_addr.ip4, IPADDR_ANY);
    dns_gethostbyname((char const*)ptr, &upgradeserver, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    if(ip4_addr_get_u32(&upgradeserver.u_addr.ip4) == IPADDR_ANY)
    {
        dns_gethostbyname((char const*)ptr, &upgradeserver, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if(ip4_addr_get_u32(&upgradeserver.u_addr.ip4) == IPADDR_ANY)
    {
        goto upgradeopt;
    }
    ESP_LOGI(TAG, "upgrade server ip got: %s", ip4addr_ntoa(&upgradeserver.u_addr.ip4));
    memset(url_buf, 0, sizeof(url_buf));
    sprintf(url_buf, "%s%s%s%s%s%s", "http://", ip4addr_ntoa(&upgradeserver.u_addr.ip4), ":8050/firmware/download?productName=", DEVICE_TYPE, "&firmwareVersion=", lValueToSend);
    config.url = url_buf;
    ESP_LOGI(TAG, "URL = %s", config.url);
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
        goto upgradeopt;
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#if 0
void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

#if CONFIG_EXAMPLE_CONNECT_WIFI
    /* Ensure to disable any WiFi power save mode, this allows best throughput
     * and hence timings for overall OTA operation.
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

    xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
}
#endif

void upgradeto(const char* version)
{
    ESP_LOGI(TAG, "Update to %s version ...", version);
    xQueueSendToBack( firmwareupdateversion, &version, 0 );
}
