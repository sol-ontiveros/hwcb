
#ifndef UART_H
#define UART_H


//including here so I don't have to pass on uart_write() (etc.) functions.
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static QueueHandle_t uart_queue; //used to define interrupt vector and handling.
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

esp_err_t sw_ctrl_uart_init();


#endif
