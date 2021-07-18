#ifndef __LEDS_HC595_H__
#define __LEDS_HC595_H__

#define HC595_LED0      (0x0001 << 0)
#define HC595_LED1      (0x0001 << 1)
#define HC595_LED2      (0x0001 << 2)
#define HC595_LED3      (0x0001 << 3)
#define HC595_LED4      (0x0001 << 4)
#define HC595_LED5      (0x0001 << 5)
#define HC595_LED6      (0x0001 << 6)
#define HC595_LED7      (0x0001 << 7)
#define HC595_LED8      (0x0001 << 8)
#define HC595_LED9      (0x0001 << 9)
#define HC595_LED10     (0x0001 << 10)
#define HC595_LED11     (0x0001 << 11)
#define HC595_LED12     (0x0001 << 12)
#define HC595_LED13     (0x0001 << 13)
#define HC595_LED14     (0x0001 << 14)
#define HC595_LED15     (0x0001 << 15)
#define HC595_LED_ALL   (0xFFFF)

void led_poweron(void);
void led_on(unsigned short led, bool on);
void rs485tx_led_blink(void);
void rs485rx_led_blink(void);
void data_report_led_blink(void);
void work_online_led_blink(void);
void work_offline_led_blink(void);
void work_upgrade_led_blink(void);
void work_online_led_stop(void);
void work_offline_led_stop(void);
void work_upgrade_led_stop(void);
#endif