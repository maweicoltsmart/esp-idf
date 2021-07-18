#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "param.h"
#include "freertos/FreeRTOS.h"
#include "string.h"
#include "crc32.h"

#define STORAGE_NAMESPACE "pragma"
#define DEVICE_INFORMATION_KEY    "devinfo"
#define WORK_PRAGMA_KEY    "workpragma"

char *const strworkmode[] = {"MJCloud", "TCPServer", "TCPClient", "UDPServer", "UDPClient", "HTTPClient", "VSPD", "ModbusTCP->RTU", "ModbusTCP->ASCII", "ModbusRTU->TCP", "ModbusASCII->TCP", "MQTT"};
char *const strflowctrl[] = {"None", "Hardware", "XonXoff"};
char *const strparity[] = {"None", "Odd", "Even", "Mark", "Space"};
char *const strstopbit[] = {"1", "2"};

#pragma pack(4)
st_Work_Pragma stWork_Pragma;
st_Device_Info stDevice_Info;
#pragma pack()

esp_err_t nvs_write(const char *key, void *data, int bytes)
{
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  // Read the size of memory space required for blob
  size_t required_size = bytes;  // value will default to 0, if not set yet in NVS

  err = nvs_set_blob(my_handle, key, data, required_size);
  if (err != ESP_OK) return err;

  // Commit
  err = nvs_commit(my_handle);
  if (err != ESP_OK) return err;

  // Close
  nvs_close(my_handle);
  return ESP_OK;
}

esp_err_t nvs_read(const char *key, void *data, int bytes)
{
  nvs_handle_t my_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) return err;

  // Read the size of memory space required for blob
  size_t required_size = bytes;  // value will default to 0, if not set yet in NVS

  err = nvs_get_blob(my_handle, key, data, &required_size);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

  // Close
  nvs_close(my_handle);
  return ESP_OK;
}

void devinfoinit(void)
{
  stDevice_Info.devid[0] = 0x20;
  stDevice_Info.devid[1] = 0x19;
  stDevice_Info.devid[2] = 0x11;
  stDevice_Info.devid[3] = 0x15;
  stDevice_Info.devid[4] = 0x00;
  stDevice_Info.devid[5] = 0x00;
  stDevice_Info.devid[6] = 0x00;
  stDevice_Info.devid[7] = 0x01;// 此值在生产测试环节会用到不要改

  stDevice_Info.crc32 = crc32((const char*)&stDevice_Info, sizeof(stDevice_Info) - 4);

  if(ESP_OK != nvs_write(DEVICE_INFORMATION_KEY, &stDevice_Info, sizeof(stDevice_Info)))
  {
    printf("devinfo nvs error\r\n");
  }
}

void devinforestore(void)
{
  stDevice_Info.crc32 = crc32((const char*)&stDevice_Info, sizeof(stDevice_Info) - 4);

  if(ESP_OK != nvs_write(DEVICE_INFORMATION_KEY, &stDevice_Info, sizeof(stDevice_Info)))
  {
    printf("devinfo nvs error\r\n");
  }
}

void getdevinfo(void)
{
  esp_err_t err;

  err = nvs_read(DEVICE_INFORMATION_KEY, &stDevice_Info, sizeof(stDevice_Info));
  if((err != ESP_OK) || (stDevice_Info.crc32 != crc32((const char*)&stDevice_Info, sizeof(stDevice_Info) - 4)))
  {
    printf("[ERROR:] nvs error! system init device info by default!\r\n");
    devinfoinit();
  }
  else
  {
    printf("get device info success\r\n");
  }
}

void factoryresetworkpragma(void)
{
  memset(stWork_Pragma.stRemote_IP.hostname,0,HOST_NAME_MAX_LEN);
  memcpy(stWork_Pragma.stRemote_IP.hostname,DEFAULT_REMOTE_HOST_NAME,strlen(DEFAULT_REMOTE_HOST_NAME));
  ip4_addr_set_u32(&stWork_Pragma.stRemote_IP.remoteip.u_addr.ip4, IPADDR_ANY);
  stWork_Pragma.stRemote_IP.remoteport = DEFAULT_REMOTE_PORT;
  stWork_Pragma.stRebootWhenTimeout.netidletimeout = 0;

  stWork_Pragma.stUart_Pragma.baud = 9600;
  stWork_Pragma.stUart_Pragma.databit = 8;
  stWork_Pragma.stUart_Pragma.stopbit = 1;
  stWork_Pragma.stUart_Pragma.parity = PRAGMA_UART_PARITY_NONE;
  stWork_Pragma.stUart_Pragma.flowctrl = PRAGMA_UART_FLOWCTRL_NONE;
  stWork_Pragma.stRebootWhenTimeout.serialidletimeout = 0;

  memset(stWork_Pragma.stModem_Pragma.apn, 0, sizeof(stWork_Pragma.stModem_Pragma.apn));
  memset(stWork_Pragma.stModem_Pragma.username, 0, sizeof(stWork_Pragma.stModem_Pragma.username));
  memset(stWork_Pragma.stModem_Pragma.password, 0, sizeof(stWork_Pragma.stModem_Pragma.password));

  memset(stWork_Pragma.stExternPragma.stMQTTClient.stUser.username, 0, sizeof(stWork_Pragma.stExternPragma.stMQTTClient.stUser.username));
  strcpy(stWork_Pragma.stExternPragma.stMQTTClient.stUser.username, "18701872013");
  memset(stWork_Pragma.stExternPragma.stMQTTClient.stUser.password, 0, sizeof(stWork_Pragma.stExternPragma.stMQTTClient.stUser.password));
  strcpy(stWork_Pragma.stExternPragma.stMQTTClient.stUser.password, "mawei19870125");
  stWork_Pragma.crc32 = crc32((const char*)&stWork_Pragma, sizeof(stWork_Pragma) - 4);
  restoreworkpragma();
}

void getworkpragma(void)
{
  esp_err_t err;

  err = nvs_read(WORK_PRAGMA_KEY, &stWork_Pragma, sizeof(stWork_Pragma));

  if((err != ESP_OK) || (stWork_Pragma.crc32 != crc32((const char*)&stWork_Pragma, sizeof(stWork_Pragma) - 4)))
  {
    printf("[ERROR:] nvs error! system do factory reset by default!\r\n");
    factoryresetworkpragma();
  }
  else
  {
    printf("get workpragma success\r\n");
  }
}

void restoreupgradeinfo(pst_UpgradeInfo pstUpgradeInfo)
{

}

void restoreworkpragma(void)
{
  stWork_Pragma.crc32 = crc32((const char*)&stWork_Pragma, sizeof(stWork_Pragma) - 4);
  if(ESP_OK != nvs_write(WORK_PRAGMA_KEY, &stWork_Pragma, sizeof(stWork_Pragma)))
  {
    printf("devinfo nvs error\r\n");
  }
}