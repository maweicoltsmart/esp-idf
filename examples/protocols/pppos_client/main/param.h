#ifndef __PRAGMA_H__
#define __PRAGMA_H__

#include <stdio.h>
#include "string.h"
#include "lwip/ip_addr.h"
#include "freertos/FreeRTOS.h"

#ifndef FALSE
#define FALSE  false
#endif

#ifndef TRUE
#define TRUE   true
#endif

//#define LWIP_TCP_KEEPALIVE      1
#define  TCP_KEEPIDLE_DEFAULT     5UL       // 5秒内连接双方都无数据，则发起保活探测（该值默认为2小时）
#define  TCP_KEEPINTVL_DEFAULT    20UL         // 每1秒发送一次保活探测
//保活机制启动后，一共发送5次保活探测包，如果这5个包对方均无回应，则表示连接异常，内核关闭连接，并发送err回调到用户程序
#define  TCP_KEEPCNT_DEFAULT      2UL
#define  TCP_MAXIDLE  TCP_KEEPCNT_DEFAULT * TCP_KEEPINTVL_DEFAULT

#define MODULE_WORK_MODE_MJ_CLOUD                       (0U)
#define MODULE_WORK_MODE_TCP_SERVER                     (1U)
#define MODULE_WORK_MODE_TCP_CLIENT                     (2U)
#define MODULE_WORK_MODE_UDP_SERVER                     (3U)
#define MODULE_WORK_MODE_UDP_CLIENT                     (4U)
#define MODULE_WORK_MODE_HTTP_CLIENT                    (5U)
#define MODULE_WORK_MODE_VSPD                           (6U)
#define MODULE_WORK_MODE_MODBUS_TCP2RTU                 (7U)
#define MODULE_WORK_MODE_MODBUS_TCP2ASCII               (8U)
#define MODULE_WORK_MODE_MODBUS_RTU2TCP                 (9U)
#define MODULE_WORK_MODE_MODBUS_ASCII2TCP               (10U)
#define MODULE_WORK_MODE_MQTT                           (11U)
#define MODULE_WORK_MODE_LIMIT                          (12U)
extern char *const strworkmode[];

#define WIFI_WORK_MODE_STA                              (0U)
#define WIFI_WORK_MODE_AP                               (1U)
#define WIFI_WORK_MODE_LIMIT                            (2U)
extern char *const strwifiworkmode[];

#define REG_PKG_MODE_NONE                               (0U)
#define REG_PKG_MODE_MAC_CONNECTED                      (1U)
#define REG_PKG_MODE_MAC_EVERY                          (2U)
#define REG_PKG_MODE_MAC_BOTH                           (3U)
#define REG_PKG_MODE_USER_CONNECTED                     (4U)
#define REG_PKG_MODE_USER_EVERY                         (5U)
#define REG_PKG_MODE_USER_BOTH                          (6U)
#define REG_PKG_MODE_LIMIT                              (7U)
extern char *const strregpkgmode[];

/* Module state code */
#define MODULE_STATE_IDLE                               (1U << 0)
#define MODULE_STATE_LISTEN                             (1U << 1)
#define MODULE_STATE_CONNECTING                         (1U << 2)
#define MODULE_STATE_CONNECTED                          (1U << 3)
#define MODULE_STATE_ERROR                              (1U << 4)

/* DHCP on or not*/
#define PRAGMA_DHCP_ON                                  (1U)
#define PRAGMA_DHCP_OFF                                 (0U)

/* UART PARITY */
#define PRAGMA_UART_PARITY_NONE                         (0U)
#define PRAGMA_UART_PARITY_ODD                          (1U)
#define PRAGMA_UART_PARITY_EVEN                         (2U)
#define PRAGMA_UART_PARITY_MARK                         (3U)
#define PRAGMA_UART_PARITY_SPACE                        (4U)
#define PRAGMA_UART_PARITY_LIMIT                        (5U)
extern char *const strparity[];
/* UART FLOWCTRL */
#define PRAGMA_UART_FLOWCTRL_NONE                       (0U)
#define PRAGMA_UART_FLOWCTRL_HARDWARE                   (1U)
#define PRAGMA_UART_FLOWCTRL_XONOFF                     (2U)
#define PRAGMA_UART_FLOWCTRL_LIMIT                      (3U)
extern char *const strflowctrl[];
/* UART stop bit */
#define PRAGMA_UART_STOP_BIT_1                          (1)
#define PRAGMA_UART_STOP_BIT_1_5                        (1.5)
#define PRAGMA_UART_STOP_BIT_2                          (2)
#define PRAGMA_UART_STOP_BIT_LIMIT                      (3U)
extern char *const strstopbit[];

#define PRAGMA_MQTT_QOS0                                (0)
#define PRAGMA_MQTT_QOS1                                (1)
#define PRAGMA_MQTT_QOS2                                (2)
#define PRAGMA_MQTT_QOS_LIMIT                           (3)
extern char *const strmqttqos[];
/* DNS Option */
#define HOST_NAME_MAX_LEN                               (30U)
#define DEFAULT_DNS_IP                                  "114.114.114.114"

#define USER_NAME_MAX_LEN                               (32 + 1)
#define PASSWORD_MAX_LEN                                (32 + 1)


/* DEFAULT REMOTE IP */
//#define DEFAULT_REMOTE_IP                               "192.168.2.100"
#define DEFAULT_REMOTE_PORT                             1883
#define DEFAULT_REMOTE_HOST_NAME                        "121.42.231.122"

#define HARDWARE_VERSION               "V1.1.0"
#define SOFTWARE_VERSION               "VERSION 1.0 ("__DATE__" "__TIME__")"

#define DEVICE_TYPE                    "MJ-DI12"

#define APN_STR_MAX_LEN                                 50
#define APN_USER_MAX_LEN                                50
#define APN_PWD_MAX_LEN                                 50

#define MQTT_ONLINE_TOPIC_LEN                           (50 + 1)
#define MQTT_ONLINE_MSG_LEN                             (50 + 1)
#define MQTT_PUB_TOPIC_LEN                              (90 + 1)
#define MQTT_SUB_TOPIC_LEN                              (90 + 1)
#define MQTT_LAST_WILL_TOPIC_LEN                        (90 + 1)
#define MQTT_LAST_WILL_MSG_LEN                          (100 + 1)

typedef struct{
  char hostname[HOST_NAME_MAX_LEN];
  ip_addr_t remoteip;
  in_port_t remoteport;
}st_Remote_IP, *pst_Remote_IP;

typedef struct{
  uint16_t netidletimeout;
  uint16_t serialidletimeout;
}st_RebootWhenTimeout, *pst_RebootWhenTimeout;

typedef struct{
  char username[USER_NAME_MAX_LEN];
  char password[PASSWORD_MAX_LEN];
}st_User, *pst_User;

typedef struct{
  st_User stUser;
  uint8_t CleanSession;
  uint32_t KeepAlive;
  /*uint8_t OnlineTopic[MQTT_ONLINE_TOPIC_LEN];
  uint8_t OnlineQOS;
  uint8_t OnlineRetain;
  uint8_t OnlineMsg[MQTT_ONLINE_MSG_LEN];*/
  char PublishTopic[MQTT_PUB_TOPIC_LEN];
  uint8_t PublishQOS;
  uint8_t PublishRetain;
  char SubTopic[MQTT_SUB_TOPIC_LEN];
  uint8_t SubQOS;
  char LastWillTopic[MQTT_LAST_WILL_TOPIC_LEN];
  uint8_t LastWillQOS;
  uint8_t LastWillRetain;
  char LastWillMsg[MQTT_LAST_WILL_MSG_LEN];
}st_MQTTClient, *pst_MQTTClient;

typedef union{
  st_MQTTClient stMQTTClient;
}st_ExternPragma, *pst_ExternPragma;

typedef struct{
  uint32_t baud;        // 600bps ~ 1Mbps
  uint8_t databit;      // 5,6,7,8
  float stopbit;      // 1,2
  uint8_t parity;       // PRAGMA_UART_PARITY_ODD,PRAGMA_UART_PARITY_EVEN,PRAGMA_UART_PARITY_MARK,PRAGMA_UART_PARITY_CLEAR
  uint8_t flowctrl;
}st_Uart_Pragma, *pst_Uart_Pragma;

typedef struct{
  char apn[APN_STR_MAX_LEN + 1];
  char username[APN_USER_MAX_LEN + 1];
  char password[APN_PWD_MAX_LEN + 1];
}st_Modem_Pragma, *pst_Modem_Pragma;

typedef struct{
  st_Modem_Pragma stModem_Pragma;
  st_Uart_Pragma stUart_Pragma;
  st_Remote_IP stRemote_IP;
  st_RebootWhenTimeout stRebootWhenTimeout;
  st_ExternPragma stExternPragma;
  uint32_t crc32;
}st_Work_Pragma, *pst_Work_Pragma;

typedef struct{
  uint8_t devid[8];
  uint32_t crc32;
}st_Device_Info, *pst_Device_Info;

typedef struct{
  //uint8_t macaddr[6];
  uint8_t majorversion;
  uint8_t minorversion;
  uint32_t crc32;
}st_UpgradeInfo, *pst_UpgradeInfo;

extern st_Work_Pragma stWork_Pragma;
extern st_Device_Info stDevice_Info;

extern void factoryset(void);
extern void getdevinfo(void);
extern void factoryresetworkpragma(void);
extern void getworkpragma(void);
extern void restoreworkpragma(void);
extern void devinforestore(void);
#endif