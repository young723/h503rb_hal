
#ifndef _QST_LOG_H_
#define _QST_LOG_H_

#include "stdio.h"
#include <stdarg.h>

//#define QST_LOG_USE_CUSTOM_FUNC
#define QST_LOG_USE_STD_PRINTF
//#define QST_LOG_USE_KEIL_PRINTF
//#define QST_LOG_USE_SEGGER_RTT
//#define QST_LOG_USE_ESP8266

#if defined(QST_LOG_USE_CUSTOM_FUNC)
extern int qst_printf(const char *format, ...);
#define qst_logi(fmt, args...)		qst_printf(fmt, ##args)
#define qst_loge(fmt, args...)		qst_printf("error:"fmt, ##args)
#endif

#if defined(QST_LOG_USE_STD_PRINTF)
//#define qst_logi(fmt, args...)		printf(fmt"%s", ##args, "\r\n")
#define qst_logi(fmt, args...)			printf(fmt, ##args)
#define qst_loge(fmt, args...)			printf("error: %s line:%d "fmt, __func__, __LINE__, ##args)
#endif

#if defined(QST_LOG_USE_KEIL_PRINTF)
extern int keil_printf(char *fmt, ...);
#define qst_logi(fmt, args...)          keil_printf(fmt, ##args)
#define qst_loge(fmt, args...)          keil_printf("error:"fmt, ##args)
#endif

#if defined(SEGGER_RTT_SUPPORT)
#include "SEGGER_RTT.h"
#define qst_logi(fmt, args...)          SEGGER_RTT_printf(0, fmt, ##args);
#define qst_loge(fmt, args...)          SEGGER_RTT_printf(0, fmt, ##args);
#endif

#if !defined(QST_LOG_USE_CUSTOM_FUNC)&&!defined(QST_LOG_USE_STD_PRINTF)&&!defined(QST_LOG_USE_KEIL_PRINTF)&&!defined(QST_LOG_USE_SEGGER_RTT)
#define qst_logi(fmt, args...)
#define qst_loge(fmt, args...)
#endif

#if defined(QST_LOG_USE_ESP8266)
extern int esp8266_printf(char *fmt, ...);
#define qst_logi(fmt, args...)          esp8266_printf(fmt, ##args)
#endif

#endif

