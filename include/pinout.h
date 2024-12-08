#ifndef _H_PINOUT_
#define _H_PINOUT_

#define LED_ERR 13
#define LED_SAP 14
#define LED_REC 27

#define SD_CS 8
#define SD_MISO 9
#define SD_MOSI 7
#define SD_SCLK 10

#define RS485_RX 19
#define RS485_TX 22
#define RS485_TNOW 21

#define IRDA_TX 33
#define IRDA_RX 25
#define IRDA_nTNOW 26

#define DEBUG_SWITCH 15

#define TEMPSENSOR 4

#ifdef ENABLE_RTC
#define RTC_SCL 32
#define RTC_SDA 20
#endif

#endif