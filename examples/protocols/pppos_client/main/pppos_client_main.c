/* PPPoS Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "mqtt_client.h"
#include "esp_modem.h"
#include "esp_modem_netif.h"
#include "esp_log.h"
#include "sim800.h"
#include "bg96.h"
#include "sim7600.h"
#include "leds.h"
#include "inputio.h"
#include "flow_meter.h"
#include "param.h"
#include "lwip/dns.h"
#include "mydns.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "hex.h"
#include "global_variable.h"


static const char *TAG = "pppos_example";
EventGroupHandle_t event_group = NULL;
const int CONNECT_BIT = BIT0;
const int STOP_BIT = BIT1;
const int GOT_DATA_BIT = BIT2;
const int DATA_REPORT_BIT = BIT3;
static modem_dce_t *ppp_dce = NULL;
int32_t LTE_RSSI = 99;
uint32_t temperature = 0;

#if CONFIG_EXAMPLE_SEND_MSG
/**
 * @brief This example will also show how to send short message using the infrastructure provided by esp modem library.
 * @note Not all modem support SMG.
 *
 */
static esp_err_t example_default_handle(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    }
    return err;
}

static esp_err_t example_handle_cmgs(modem_dce_t *dce, const char *line)
{
    esp_err_t err = ESP_FAIL;
    if (strstr(line, MODEM_RESULT_CODE_SUCCESS)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_SUCCESS);
    } else if (strstr(line, MODEM_RESULT_CODE_ERROR)) {
        err = esp_modem_process_command_done(dce, MODEM_STATE_FAIL);
    } else if (!strncmp(line, "+CMGS", strlen("+CMGS"))) {
        err = ESP_OK;
    }
    return err;
}

#define MODEM_SMS_MAX_LENGTH (128)
#define MODEM_COMMAND_TIMEOUT_SMS_MS (120000)
#define MODEM_PROMPT_TIMEOUT_MS (10)

static esp_err_t example_send_message_text(modem_dce_t *dce, const char *phone_num, const char *text)
{
    modem_dte_t *dte = dce->dte;
    dce->handle_line = example_default_handle;
    /* Set text mode */
    if (dte->send_cmd(dte, "AT+CMGF=1\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "set message format failed");
        goto err;
    }
    ESP_LOGD(TAG, "set message format ok");
    /* Specify character set */
    dce->handle_line = example_default_handle;
    if (dte->send_cmd(dte, "AT+CSCS=\"GSM\"\r", MODEM_COMMAND_TIMEOUT_DEFAULT) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "set character set failed");
        goto err;
    }
    ESP_LOGD(TAG, "set character set ok");
    /* send message */
    char command[MODEM_SMS_MAX_LENGTH] = {0};
    int length = snprintf(command, MODEM_SMS_MAX_LENGTH, "AT+CMGS=\"%s\"\r", phone_num);
    /* set phone number and wait for "> " */
    dte->send_wait(dte, command, length, "\r\n> ", MODEM_PROMPT_TIMEOUT_MS);
    /* end with CTRL+Z */
    snprintf(command, MODEM_SMS_MAX_LENGTH, "%s\x1A", text);
    dce->handle_line = example_handle_cmgs;
    if (dte->send_cmd(dte, command, MODEM_COMMAND_TIMEOUT_SMS_MS) != ESP_OK) {
        ESP_LOGE(TAG, "send command failed");
        goto err;
    }
    if (dce->state != MODEM_STATE_SUCCESS) {
        ESP_LOGE(TAG, "send message failed");
        goto err;
    }
    ESP_LOGD(TAG, "send message ok");
    return ESP_OK;
err:
    return ESP_FAIL;
}
#endif

static void modem_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ESP_MODEM_EVENT_PPP_START:
        ESP_LOGI(TAG, "Modem PPP Started");
        break;
    case ESP_MODEM_EVENT_PPP_STOP:
        ESP_LOGI(TAG, "Modem PPP Stopped");
        xEventGroupSetBits(event_group, STOP_BIT);
        break;
    case ESP_MODEM_EVENT_UNKNOWN:
        ESP_LOGW(TAG, "Unknow line received: %s", (char *)event_data);
        break;
    default:
        break;
    }
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char devidstr[17];
    char onlinetopic[50] = {'\0'};
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        work_upgrade_led_stop();
        work_offline_led_stop();
        work_online_led_blink();
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        /* /dev/【设备ID】/upgrade */
        memset(devidstr, 0, sizeof(devidstr));
        puthex((uint8_t *)devidstr, stDevice_Info.devid, 8);
        sprintf(onlinetopic, "/dev/%s/upgrade", devidstr);
        msg_id = esp_mqtt_client_subscribe(client, onlinetopic, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        work_upgrade_led_stop();
        work_online_led_stop();
        work_offline_led_blink();
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        esp_restart();
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        /*{
          "device_id":"202106150001",
          "version":"V1.0"      // 设备当前固件版本号
          "IMEI":"1234567890"   // 国际移动设备识别码（International Mobile Equipment Identity)
          "IMSI":"1234567890"   // 国际移动用户识别码（International Mobile Subscriber Identity）
          "ICCID":"1234567890"  // SIM卡卡号 (Integrate circuit card identity)
          "GPS":"116.403981,39.915101" // 全球定位坐标
        }*/
        /*/dev/【设备ID】/online*/
        memset(devidstr, 0, sizeof(devidstr));
        puthex((uint8_t *)devidstr, stDevice_Info.devid, 8);
        cJSON *root = NULL;
        root = cJSON_CreateObject();
        //cJSON_AddStringToObject(root, "device_id", devidstr);
        const esp_app_desc_t *app_desc = esp_ota_get_app_description();
        cJSON_AddStringToObject(root, "version", app_desc->version);
        cJSON_AddStringToObject(root, "IMEI", ppp_dce->imei);
        cJSON_AddStringToObject(root, "IMSI", ppp_dce->imsi);
        cJSON_AddStringToObject(root, "ICCID", ppp_dce->iccid);
        cJSON_AddStringToObject(root, "GPS", ppp_dce->location);
        cJSON_AddNumberToObject(root, "RSSI", LTE_RSSI);

        char *out = cJSON_Print(root);
        sprintf(onlinetopic, "/dev/%s/online", devidstr);
        ESP_LOGI(TAG, "Online topic = %s, message = %s", onlinetopic, out);
        msg_id = esp_mqtt_client_publish(client, onlinetopic, out, 0, 2, 0);
        ESP_LOGI(TAG, "sent publish online message successful, msg_id=%d", msg_id);
        free(out);
        cJSON_Delete(root);

        //extern void upgradeto(const char* version);
        //upgradeto("2.0");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        xEventGroupSetBits(event_group, GOT_DATA_BIT);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "MQTT other event id: %d", event->event_id);
        break;
    }
    return ESP_OK;
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP state changed event %d", event_id);
    if (event_id == NETIF_PPP_ERRORUSER) {
        /* User interrupted event from esp-netif */
        esp_netif_t *netif = *(esp_netif_t**)event_data;
        ESP_LOGI(TAG, "User interrupted event from netif:%p", netif);
    }
}


static void on_ip_event(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "IP event! %d", event_id);
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        esp_netif_dns_info_t dns_info;

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_t *netif = event->esp_netif;

        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
        esp_netif_get_dns_info(netif, 0, &dns_info);
        ESP_LOGI(TAG, "Name Server1: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        esp_netif_get_dns_info(netif, 1, &dns_info);
        ESP_LOGI(TAG, "Name Server2: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(event_group, CONNECT_BIT);

        ESP_LOGI(TAG, "GOT ip event!!!");
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
    } else if (event_id == IP_EVENT_GOT_IP6) {
        ESP_LOGI(TAG, "GOT IPv6 event!");

        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6 address " IPV6STR, IPV62STR(event->ip6_info.ip));
    }
}

void app_main(void)
{
    const esp_app_desc_t *app_desc = esp_ota_get_app_description();
    ESP_LOGI(TAG, "fireware version is %s", app_desc->version);
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
    getdevinfo();
    getworkpragma();
    inputio_init();
    led_poweron();
    work_offline_led_blink();
    xTaskCreate(&flow_meter_task, "flow_meter_task", 8192, NULL, 5, NULL);
    xTaskCreate(&input_io_task, "input_io_task", 4096, NULL, 5, NULL);
    extern void simple_ota_example_task(void *pvParameter);
    xTaskCreate(&simple_ota_example_task, "ota_example_task", 4096, NULL, 5, NULL);
#if CONFIG_LWIP_PPP_PAP_SUPPORT
    esp_netif_auth_type_t auth_type = NETIF_PPP_AUTHTYPE_PAP;
#elif CONFIG_LWIP_PPP_CHAP_SUPPORT
    esp_netif_auth_type_t auth_type = NETIF_PPP_AUTHTYPE_CHAP;
#elif !defined(CONFIG_EXAMPLE_MODEM_PPP_AUTH_NONE)
#error "Unsupported AUTH Negotiation"
#endif
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, &on_ppp_changed, NULL));

    event_group = xEventGroupCreate();

    /* create dte object */
    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    config.tx_io_num = CONFIG_EXAMPLE_MODEM_UART_TX_PIN;
    config.rx_io_num = CONFIG_EXAMPLE_MODEM_UART_RX_PIN;
    config.rts_io_num = CONFIG_EXAMPLE_MODEM_UART_RTS_PIN;
    config.cts_io_num = CONFIG_EXAMPLE_MODEM_UART_CTS_PIN;
    config.rx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE;
    config.tx_buffer_size = CONFIG_EXAMPLE_MODEM_UART_TX_BUFFER_SIZE;
    config.pattern_queue_size = CONFIG_EXAMPLE_MODEM_UART_PATTERN_QUEUE_SIZE;
    config.event_queue_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_QUEUE_SIZE;
    config.event_task_stack_size = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_STACK_SIZE;
    config.event_task_priority = CONFIG_EXAMPLE_MODEM_UART_EVENT_TASK_PRIORITY;
    config.line_buffer_size = CONFIG_EXAMPLE_MODEM_UART_RX_BUFFER_SIZE / 2;

    modem_dte_t *dte = esp_modem_dte_init(&config);
    /* Register event handler */
    ESP_ERROR_CHECK(esp_modem_set_event_handler(dte, modem_event_handler, ESP_EVENT_ANY_ID, NULL));

    // Init netif object
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *esp_netif = esp_netif_new(&cfg);
    assert(esp_netif);

    void *modem_netif_adapter = esp_modem_netif_setup(dte);
    esp_modem_netif_set_default_handlers(modem_netif_adapter, esp_netif);

    while (1) {
        modem_dce_t *dce = NULL;
        /* create dce object */
#if CONFIG_EXAMPLE_MODEM_DEVICE_SIM800
        dce = sim800_init(dte);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_BG96
        dce = bg96_init(dte);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM7600
        dce = sim7600_init(dte);
#else
#error "Unsupported DCE"
#endif
        assert(dce != NULL);
        ESP_ERROR_CHECK(dce->set_flow_ctrl(dce, MODEM_FLOW_CONTROL_NONE));
        ESP_ERROR_CHECK(dce->store_profile(dce));
        ppp_dce = dce;
        /* Print Module ID, Operator, IMEI, IMSI */
        ESP_LOGI(TAG, "Module: %s", dce->name);
        ESP_LOGI(TAG, "Operator: %s", dce->oper);
        ESP_LOGI(TAG, "IMEI: %s", dce->imei);
        ESP_LOGI(TAG, "IMSI: %s", dce->imsi);
        ESP_LOGI(TAG, "ICCID: %s", dce->iccid);
        ESP_LOGI(TAG, "GPS: %s", dce->location);
        /* Get signal quality */
        uint32_t rssi = 0, ber = 0;
        ESP_ERROR_CHECK(dce->get_signal_quality(dce, &rssi, &ber));
        switch(rssi)
        {
            case 0:
                LTE_RSSI = -113;
                break;
            case 1:
                LTE_RSSI = -111;
                break;
            case 32:
                LTE_RSSI = -51;
                break;
            case 99:
                LTE_RSSI = 99;
                break;
            default:
                LTE_RSSI = (int32_t)rssi * 2 - 113 ;
                break;
        }
        ESP_LOGI(TAG, "rssi: %d, ber: %d", LTE_RSSI, ber);

        /* Get battery voltage */
        uint32_t voltage = 0, bcs = 0, bcl = 0;
        ESP_ERROR_CHECK(dce->get_battery_status(dce, &bcs, &bcl, &voltage));
        ESP_LOGI(TAG, "Battery voltage: %d mV", voltage);
        /* setup PPPoS network parameters */
#if !defined(CONFIG_EXAMPLE_MODEM_PPP_AUTH_NONE) && (defined(CONFIG_LWIP_PPP_PAP_SUPPORT) || defined(CONFIG_LWIP_PPP_CHAP_SUPPORT))
        esp_netif_ppp_set_auth(esp_netif, auth_type, CONFIG_EXAMPLE_MODEM_PPP_AUTH_USERNAME, CONFIG_EXAMPLE_MODEM_PPP_AUTH_PASSWORD);
#endif
        /* attach the modem to the network interface */
        esp_netif_attach(esp_netif, modem_netif_adapter);
        /* Wait for IP address */
        xEventGroupWaitBits(event_group, CONNECT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        ip_addr_t dnsaddr;
        ip4addr_aton("114.114.114.114", &dnsaddr.u_addr.ip4);
        dns_setserver(1, &dnsaddr);
        TickType_t readserialwhilewaittime;
        if(ip4addr_aton((char const*)stWork_Pragma.stRemote_IP.hostname, &stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4) == 0)
        {
            ip4_addr_set_u32(&stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4, IPADDR_ANY);
            while(ip4_addr_get_u32(&stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4) == IPADDR_ANY)
            {
                get_remote_ip();
                readserialwhilewaittime = GetCurrentTime();
                while(TimerGetElapsedTime(readserialwhilewaittime) < (pdMS_TO_TICKS(2 * 1000)))
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        char devidstr[17];
        char onlinetopic[50] = {'\0'};
        /*/dev/【设备ID】/update*/
        memset(devidstr, 0, sizeof(devidstr));
        puthex((uint8_t *)devidstr, stDevice_Info.devid, 8);
        static char host[IP4ADDR_STRLEN_MAX] = {0};
        strcpy(host, ipaddr_ntoa(&stWork_Pragma.stRemote_IP.remoteip));
        /* Config MQTT */
        esp_mqtt_client_config_t mqtt_config = {
            .uri = NULL,
            .host = host,
            .port = stWork_Pragma.stRemote_IP.remoteport,
            .event_handle = mqtt_event_handler,
            .username = "18701872013",
            .password = "mawei19870125",
        };
        static char willtopic[50] = {'\0'};
        sprintf(willtopic, "/dev/%s/offline", devidstr);
        cJSON *root_offline_msg = NULL;
        root_offline_msg = cJSON_CreateObject();
        cJSON_AddStringToObject(root_offline_msg, "code", "timeout");
        mqtt_config.lwt_msg = cJSON_Print(root_offline_msg);
        //free(out);
        cJSON_Delete(root_offline_msg);
        mqtt_config.lwt_qos = 2;
        mqtt_config.lwt_topic = willtopic;
        //mqtt_config.lwt_msg = willmsg;
        mqtt_config.lwt_retain = 0;
        mqtt_config.lwt_msg_len = strlen(mqtt_config.lwt_msg);
        mqtt_config.disable_clean_session = pdFALSE;
        mqtt_config.keepalive = 120;
        mqtt_config.disable_auto_reconnect = pdTRUE;
        mqtt_config.reconnect_timeout_ms = 0;
        ESP_LOGI(TAG, "host: %s\nwilltopic: %s\nwillmsg: %s\n", host, willtopic, mqtt_config.lwt_msg);
        esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_config);
        esp_mqtt_client_start(mqtt_client);

        while(pdTRUE)
        {
            EventBits_t uxBits = xEventGroupWaitBits(event_group, DATA_REPORT_BIT | STOP_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
            if( ( uxBits & STOP_BIT ) != 0 )
            {
                // xEventGroupWaitBits() returned because just STOP_BIT was set.
                esp_restart();
            }
            else//if( ( uxBits & DATA_REPORT_BIT ) != 0 )
            {
                // xEventGroupWaitBits() returned because just DATA_REPORT_BIT was set.
                /*{
                  "device_id":"202106150001", // 采集设备的ID
                  "data":{
                   "port1":true, // 该采集设备端口1所接设备的运行状态，true：启动；false：停止
                   "port2":false,
                   "port3":false,
                   "port4":true,
                   "port5":true,
                   "port6":false,
                   "port7":false,
                   "port8":true,
                   "port9":true,
                   "port10":false,
                   "port11":false,
                   "port12":true,
                   "flow_rate":1.0, // 流量计的瞬时流量
                   "flow_total":1.0, // 流量计的正向累计总量
                   "time":"2020-06-28 17:00:00"
                  }
                 }*/
                cJSON *root = NULL;
                //cJSON *data = NULL;
                root = cJSON_CreateObject();
                //cJSON_AddStringToObject(root, "device_id", devidstr);
                //cJSON_AddItemToObject(root, "data", data = cJSON_CreateObject());
                unsigned short io_value = input_pin_read();
                float flow_rate = 0;
                double flow_total = 0;
                flow_meter_data_get(&flow_rate, &flow_total);
                cJSON_AddBoolToObject(root, "port1", (io_value & (0x0001 << 0))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port2", (io_value & (0x0001 << 1))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port3", (io_value & (0x0001 << 2))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port4", (io_value & (0x0001 << 3))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port5", (io_value & (0x0001 << 4))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port6", (io_value & (0x0001 << 5))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port7", (io_value & (0x0001 << 6))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port8", (io_value & (0x0001 << 7))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port9", (io_value & (0x0001 << 8))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port10", (io_value & (0x0001 << 9))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port11", (io_value & (0x0001 << 10))?pdTRUE:pdFALSE);
                cJSON_AddBoolToObject(root, "port12", (io_value & (0x0001 << 11))?pdTRUE:pdFALSE);
                cJSON_AddNumberToObject(root, "flow_rate", flow_rate);
                cJSON_AddNumberToObject(root, "flow_total", flow_total);
                //cJSON_AddNumberToObject(root, "temperature", temperature);
                char *out = cJSON_Print(root);
                sprintf(onlinetopic, "/dev/%s/update", devidstr);
                ESP_LOGI(TAG, "Report topic = %s, message = %s", onlinetopic, out);
                int msg_id = esp_mqtt_client_publish(mqtt_client, onlinetopic, out, 0, 2, 0);
                ESP_LOGI(TAG, "sent publish message successful, msg_id=%d", msg_id);
                free(out);
                cJSON_Delete(root);
                data_report_led_blink();
            }
        }

        esp_mqtt_client_destroy(mqtt_client);

        /* Exit PPP mode */
        ESP_ERROR_CHECK(esp_modem_stop_ppp(dte));

        xEventGroupWaitBits(event_group, STOP_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
#if CONFIG_EXAMPLE_SEND_MSG
        const char *message = "Welcome to ESP32!";
        ESP_ERROR_CHECK(example_send_message_text(dce, CONFIG_EXAMPLE_SEND_MSG_PEER_PHONE_NUMBER, message));
        ESP_LOGI(TAG, "Send send message [%s] ok", message);
#endif
        /* Power down module */
        ESP_ERROR_CHECK(dce->power_down(dce));
        ESP_LOGI(TAG, "Power down");
        ESP_ERROR_CHECK(dce->deinit(dce));

        ESP_LOGI(TAG, "Restart after 60 seconds");
        vTaskDelay(pdMS_TO_TICKS(60000));
    }


    /* Unregister events, destroy the netif adapter and destroy its esp-netif instance */
    esp_modem_netif_clear_default_handlers(modem_netif_adapter);
    esp_modem_netif_teardown(modem_netif_adapter);
    esp_netif_destroy(esp_netif);

    ESP_ERROR_CHECK(dte->deinit(dte));
}
