// Copyright 2016-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h"  // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"
#include "leds.h"
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NGHTTP_PLATFORM_HTONS(_n)  ((uint16_t)((((_n) & 0xff) << 8) | (((_n) >> 8) & 0xff)))
#define NGHTTP_PLATFORM_HTONL(_n)  ((uint32_t)( (((_n) & 0xff) << 24) | (((_n) & 0xff00) << 8) | (((_n) >> 8)  & 0xff00) | (((_n) >> 24) & 0xff) ))

#define htons(x) NGHTTP_PLATFORM_HTONS(x)
#define ntohs(x) NGHTTP_PLATFORM_HTONS(x)
#define htonl(x) NGHTTP_PLATFORM_HTONL(x)
#define ntohl(x) NGHTTP_PLATFORM_HTONL(x)

#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (2000)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_RATE_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (300)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_RATE_MS)

#define MASTER_TAG "MASTER_TEST"

#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_FLOW_RATE = 0,
    CID_TOTAL_INT_LOW,
    CID_TOTAL_POINT,
    CID_TOTAL_INT_HIGH,
    CID_COUNT
};

static SemaphoreHandle_t semp_flow_meter;
union u32bytes{
    uint8_t bytes[4];
    uint32_t data;
};

union floatbytes{
    uint8_t bytes[4];
    float data;
};
static union floatbytes value_flow_rate;
static double value_flow_total;
static union u32bytes value_flow_total_int_high;
static union u32bytes value_flow_total_int_low;
static union floatbytes value_flow_total_float;
// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    /*{ CID_INP_DATA_0, STR("Data_channel_0"), STR("Volts"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 0, 2,
                    INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 4, OPTS( -10, 10, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },*/
    { CID_FLOW_RATE, STR("flow_rate"), STR("%rH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0, 2,
            HOLD_OFFSET(flow_rate), PARAM_TYPE_FLOAT, 4, OPTS( 0, 0xFFFFFFFF, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    /*{ CID_INP_DATA_1, STR("Temperature_1"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 2, 2,
            INPUT_OFFSET(input_data1), PARAM_TYPE_FLOAT, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },*/
    { CID_TOTAL_INT_LOW, STR("flow_total_int_low"), STR("%rH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 3, 2,
            HOLD_OFFSET(flow_total_int_low), PARAM_TYPE_U32, 4, OPTS( 0, 0xFFFFFFFF, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    /*{ CID_INP_DATA_2, STR("Temperature_2"), STR("C"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 4, 2,
            INPUT_OFFSET(input_data2), PARAM_TYPE_FLOAT, 4, OPTS( -40, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },*/
    { CID_TOTAL_POINT, STR("flow_total_point"), STR("%rH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 5, 2,
            HOLD_OFFSET(flow_total_point), PARAM_TYPE_FLOAT, 4, OPTS( 0, 0xFFFFFFFF, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_TOTAL_INT_HIGH, STR("flow_total_int_high"), STR("%rH"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x1A, 2,
            HOLD_OFFSET(flow_total_int_high), PARAM_TYPE_U32, 4, OPTS( 0, 0xFFFFFFFF, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    /*{ CID_HOLD_TEST_REG, STR("Test_regs"), STR("__"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 10, 58,
            HOLD_OFFSET(test_regs), PARAM_TYPE_ASCII, 116, OPTS( 0, 100, 1 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_RELAY_P1, STR("RelayP1"), STR("on/off"), MB_DEVICE_ADDR1, MB_PARAM_COIL, 0, 8,
            COIL_OFFSET(coils_port0), PARAM_TYPE_U16, 2, OPTS( BIT1, 0, 0 ), PAR_PERMS_READ_WRITE_TRIGGER },
    { CID_RELAY_P2, STR("RelayP2"), STR("on/off"), MB_DEVICE_ADDR1, MB_PARAM_COIL, 8, 8,
            COIL_OFFSET(coils_port1), PARAM_TYPE_U16, 2, OPTS( BIT0, 0, 0 ), PAR_PERMS_READ_WRITE_TRIGGER }*/
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

#if !defined(bswap_16)
#  warning "Fallback on C functions for bswap_16"
static inline uint16_t bswap_16(uint16_t x)
{
    return (x >> 8) | (x << 8);
}
#endif

#if !defined(bswap_32)
#  warning "Fallback on C functions for bswap_32"
static inline uint32_t bswap_32(uint32_t x)
{
    return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
}
#endif

/* Get a float from 4 bytes (Modbus) without any conversion (ABCD) */
float modbus_get_float_abcd(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = ntohl(((uint32_t)src[0] << 16) + src[1]);
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Get a float from 4 bytes (Modbus) with swapped words (CDAB) */
float modbus_get_float_cdab(const uint16_t *src)
{
    float f;
    uint32_t i;

    i = ntohl((((uint32_t)src[1]) << 16) + src[0]);
    memcpy(&f, &i, sizeof(float));

    return f;
}

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
       switch(param_descriptor->mb_param_type)
       {
           case MB_PARAM_HOLDING:
               instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_INPUT:
               instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_COIL:
               instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
               break;
           case MB_PARAM_DISCRETE:
               instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
               break;
           default:
               instance_ptr = NULL;
               break;
       }
    } else {
        ESP_LOGE(MASTER_TAG, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    uint32_t value = 0;
    //bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(MASTER_TAG, "Start modbus test...");

    while(1){
    //for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY && (!alarm_state); retry++) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                rs485tx_led_blink();
                /*if ((param_descriptor->param_type == PARAM_TYPE_ASCII) &&
                        (param_descriptor->cid == CID_HOLD_TEST_REG)) {
                   // Check for long array of registers of type PARAM_TYPE_ASCII
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                                            (uint8_t*)temp_data_ptr, &type);
                    if (err == ESP_OK) {
                        ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%08x) read successful.",
                                                 param_descriptor->cid,
                                                 (char*)param_descriptor->param_key,
                                                 (char*)param_descriptor->param_units,
                                                 *(uint32_t*)temp_data_ptr);
                        // Initialize data of test array and write to slave
                        if (*(uint32_t*)temp_data_ptr != 0xAAAAAAAA) {
                            memset((void*)temp_data_ptr, 0xAA, param_descriptor->param_size);
                            *(uint32_t*)temp_data_ptr = 0xAAAAAAAA;
                            err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key,
                                                              (uint8_t*)temp_data_ptr, &type);
                            if (err == ESP_OK) {
                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (0x%08x), write successful.",
                                                            param_descriptor->cid,
                                                            (char*)param_descriptor->param_key,
                                                            (char*)param_descriptor->param_units,
                                                            *(uint32_t*)temp_data_ptr);
                            } else {
                                ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                                        param_descriptor->cid,
                                                        (char*)param_descriptor->param_key,
                                                        (int)err,
                                                        (char*)esp_err_to_name(err));
                            }
                        }
                    } else {
                        ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                                param_descriptor->cid,
                                                (char*)param_descriptor->param_key,
                                                (int)err,
                                                (char*)esp_err_to_name(err));
                    }
                } else */{
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                        (uint8_t*)&value, &type);
                    if (err == ESP_OK) {
                        rs485rx_led_blink();
                        if (param_descriptor->mb_param_type == MB_PARAM_HOLDING){
                            if(cid == CID_FLOW_RATE)
                            {
                                value_flow_rate.bytes[0] = (value >> 16) & 0x00FF;
                                value_flow_rate.bytes[1] = (value >> 24) & 0x00FF;
                                value_flow_rate.bytes[2] = value & 0x00FF;
                                value_flow_rate.bytes[3] = (value >> 8) & 0x00FF;

                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %f (0x%08X) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            value_flow_rate.data,
                                            *(unsigned int*)value_flow_rate.bytes);
                            }
                            else if(cid == CID_TOTAL_INT_LOW)
                            {
                                value_flow_total_int_low.bytes[0] = (value >> 16) & 0x00FF;
                                value_flow_total_int_low.bytes[1] = (value >> 24) & 0x00FF;
                                value_flow_total_int_low.bytes[2] = value & 0x00FF;
                                value_flow_total_int_low.bytes[3] = (value >> 8) & 0x00FF;
                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %d (0x%08X) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            value_flow_total_int_low.data,
                                            *(unsigned int*)value_flow_total_int_low.bytes);
                            }
                            else if(cid == CID_TOTAL_POINT)
                            {
                                value_flow_total_float.bytes[0] = (value >> 16) & 0x00FF;
                                value_flow_total_float.bytes[1] = (value >> 24) & 0x00FF;
                                value_flow_total_float.bytes[2] = value & 0x00FF;
                                value_flow_total_float.bytes[3] = (value >> 8) & 0x00FF;
                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %f (0x%08X) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            value_flow_total_float.data,
                                            *(unsigned int*)value_flow_total_float.bytes);
                            }
                            else if(cid == CID_TOTAL_INT_HIGH)
                            {
                                value_flow_total_int_high.bytes[0] = (value >> 16) & 0x00FF;
                                value_flow_total_int_high.bytes[1] = (value >> 24) & 0x00FF;
                                value_flow_total_int_high.bytes[2] = value & 0x00FF;
                                value_flow_total_int_high.bytes[3] = (value >> 8) & 0x00FF;
                                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = %d (0x%08X) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            value_flow_total_int_high.data,
                                            *(unsigned int*)value_flow_total_int_high.bytes);
                            }
                            /*if (((value > param_descriptor->param_opts.max) ||
                                (value < param_descriptor->param_opts.min))) {
                                    alarm_state = true;
                                    break;
                            }*/
                        } else {
                            /*uint16_t state = *(uint16_t*)temp_data_ptr;
                            const char* rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
                            ESP_LOGD(MASTER_TAG, "Characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (char*)param_descriptor->param_units,
                                            (const char*)rw_str,
                                            *(uint16_t*)temp_data_ptr);
                            if (state & param_descriptor->param_opts.opt1) {
                                alarm_state = true;
                                break;
                            }*/
                        }
                    } else {
                        ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                            param_descriptor->cid,
                                            (char*)param_descriptor->param_key,
                                            (int)err,
                                            (char*)esp_err_to_name(err));
                    }
                }
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        xSemaphoreTake(semp_flow_meter, portMAX_DELAY);
        value_flow_total = value_flow_total_int_high.data * 1e9 + value_flow_total_int_low.data + value_flow_total_float.data;
        xSemaphoreGive(semp_flow_meter);
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
    }

    /*if (alarm_state) {
        ESP_LOGI(MASTER_TAG, "Alarm triggered by cid #%d.",
                                        param_descriptor->cid);
    } else {
        ESP_LOGE(MASTER_TAG, "Alarm is not triggered after %d retries.",
                                        MASTER_MAX_RETRY);
    }*/
    ESP_LOGI(MASTER_TAG, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                              CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

void flow_meter_task(void)
{
    semp_flow_meter = xSemaphoreCreateMutex();
    xSemaphoreGive(semp_flow_meter);
    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    master_operation_func(NULL);
}

void flow_meter_data_get(float *flow_rate, double *flow_total)
{
    xSemaphoreTake(semp_flow_meter, portMAX_DELAY);
    *flow_rate = value_flow_rate.data;
    *flow_total = value_flow_total;
    xSemaphoreGive(semp_flow_meter);
}