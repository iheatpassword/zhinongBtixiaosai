/**
  * @file    app_report.h
  * @brief   1 Hz telemetry, OLED screens, and the shared text helpers used by
  *          the command parser.
  */
#ifndef APP_REPORT_H
#define APP_REPORT_H

#include "bsp_types.h"

void Report_Log(void);          /* 1 Hz, log task, may block */
void Report_Ui(void);           /* 5 Hz, ui task            */

void App_Puts(const char *s);   /* to USART1 + USART3 + UART5 */
void App_PutInt(int v);

#endif /* APP_REPORT_H */
