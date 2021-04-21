
/*
Convenince initialization and configuration of UART.
*/

#include <stdio.h>
#include <esp_log.h>
#include <esp_system.h>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"


#define DEFAULT_FLOWCTRL_THRESH 16
uart_port_t sw_ctrl_port = CONFIG_SW_CTRL_UART_PORT;

#ifdef CONFIG_USE_HW_FLOWCTRL
  #define UART_RTS (CONFIG_UART_RTS)
  #define UART_CTS (CONFIG_UART_CTS)
  #define USE_HW_FLOWCTRL CONFIG_USE_HW_FLOWCTRL
#else
  #define UART_RTS (UART_PIN_NO_CHANGE)
  #define UART_CTS (UART_PIN_NO_CHANGE)
  #define USE_HW_FLOWCTRL 0
#endif

static const char* TAG = "UART_Control";

uart_config_t uart_config = {
    .baud_rate = CONFIG_UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,

    //.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    //.rx_flow_ctrl_thresh = 122,
};

/*
uart_config_t uart_config = {
       .baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .source_clk = UART_SCLK_APB,
   };
*/

/*
uart_intr_config_t uart_intr_config {
  .intr_enable_mask = 0,
  .rx_timeout_thresh = 0,
  .txfifo_empty_intr_thresh = 0,
  .rxfifo_full_thresh = 0
};
*/


esp_err_t sw_ctrl_uart_init() {
  int intr_alloc_flags = 0;

  ESP_ERROR_CHECK(uart_driver_install(sw_ctrl_port, CONFIG_UART_BUFFER_SIZE * 2, CONFIG_UART_BUFFER_SIZE * 2, 0, &uart_queue, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(sw_ctrl_port, &uart_config));
  if (USE_HW_FLOWCTRL) {
    uart_set_hw_flow_ctrl(sw_ctrl_port, UART_HW_FLOWCTRL_CTS_RTS, DEFAULT_FLOWCTRL_THRESH);
  }
  ESP_ERROR_CHECK(uart_set_pin(sw_ctrl_port, CONFIG_UART_TXD, CONFIG_UART_RXD, UART_RTS, UART_CTS));

  //Set uart pattern detect function.
  //uart_enable_pattern_det_baud_intr(CONFIG_SW_CTRL_UART_PORT, '+', PATTERN_CHR_NUM, 9, 0, 0);
  //Reset the pattern queue length to record at most 20 pattern positions.
  //uart_pattern_queue_reset(CONFIG_SW_CTRL_UART_PORT, 20);

  ESP_LOGI(TAG, "Sofware control port initialized.");
  return ESP_OK;
}
