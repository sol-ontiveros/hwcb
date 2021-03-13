
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>

#include "driver/uart.h"
#include "wasp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

//UART config macros from kconfig
#define EX_UART_NUM CONFIG_SW_CTRL_UART_PORT //rename example macro
#define BUF_SIZE (CONFIG_UART_BUFFER_SIZE)
#define RD_BUF_SIZE (BUF_SIZE)
#define DEFAULT_FLOWCTRL_THRESH 16
uart_port_t sw_ctrl_port = CONFIG_SW_CTRL_UART_PORT;
static QueueHandle_t uart_queue; //used to define interrupt vector and handling.
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#ifdef CONFIG_USE_HW_FLOWCTRL
  #define UART_RTS (CONFIG_UART_RTS)
  #define UART_CTS (CONFIG_UART_CTS)
  #define USE_HW_FLOWCTRL CONFIG_USE_HW_FLOWCTRL
#else
  #define UART_RTS (UART_PIN_NO_CHANGE)
  #define UART_CTS (UART_PIN_NO_CHANGE)
  #define USE_HW_FLOWCTRL 0
#endif


static const char* TAG = "HWCB_MAIN";



/*************** UART INIT and CONFIG *************************************
****************************************************************************/
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

esp_err_t sw_ctrl_uart_init() {
  ESP_ERROR_CHECK(uart_param_config(sw_ctrl_port, &uart_config));
  if (USE_HW_FLOWCTRL) {
    uart_set_hw_flow_ctrl(sw_ctrl_port, UART_HW_FLOWCTRL_CTS_RTS, DEFAULT_FLOWCTRL_THRESH);
  }
  ESP_ERROR_CHECK(uart_set_pin(sw_ctrl_port, CONFIG_UART_TXD, CONFIG_UART_RXD, UART_RTS, UART_CTS));
  ESP_ERROR_CHECK(uart_driver_install(sw_ctrl_port, CONFIG_UART_BUFFER_SIZE * 2, CONFIG_UART_BUFFER_SIZE * 2, 20, &uart_queue, 0));

  //Set uart pattern detect function.
  //uart_enable_pattern_det_baud_intr(CONFIG_SW_CTRL_UART_PORT, '+', PATTERN_CHR_NUM, 9, 0, 0);
  //Reset the pattern queue length to record at most 20 pattern positions.
  //uart_pattern_queue_reset(CONFIG_SW_CTRL_UART_PORT, 20);

  ESP_LOGI(TAG, "Sofware control port initialized.");
  return ESP_OK;
}
/*******************************************************************************
*******************************************************************************/




/************************************************************************************************
UART interrupt handler setup. This should call the wasp rx/parse routines on data receipt,
and report all other UART errors. This event must match with the setup in the helper function in
UART.c
**************************************************************************************************/
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    wasp_handle_t* wasp_handle = pvParameters;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "UART[%d] event:", wasp_handle->config.uart_port);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA EVT]: Started. Size: %d", event.size);

                    //This is the main receive handler for wasp data frames.
                    wasp_rx_routine(wasp_handle);

                    ESP_LOGI(TAG, "[UART DATA EVT]: Complete");
                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGW(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                //TODO
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(wasp_handle->config.uart_port, &buffered_size);
                    int pos = uart_pattern_pop_pos(wasp_handle->config.uart_port);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    uart_flush_input(wasp_handle->config.uart_port);
                    /*if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    } */
                    break;
                //Others
                default:
                    ESP_LOGW(TAG, "Unhandled uart event - Type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
/**********************************************************************************************************
***********************************************************************************************************/





/***********************************************************************************************************
****************************************** WASP COMMAND HANDLERS *******************************************
***********************************************************************************************************/

esp_err_t simple_handler(wasp_handle_t* handle, wasp_frame_t* frame) {
  printf("This is the handler: %i\n", frame->cmd);
  return ESP_OK;
}

const wasp_command_function_t simple = {
  .command = "Simple",
  .cmd_hex_code = 0x10,
  .handler = simple_handler
};

/*************************************************************************************************************
**************************************************************************************************************/




/******************************************************************************
******************************** MAIN *****************************************
******************************************************************************/
void app_main(void)
{
  sw_ctrl_uart_init();

  wasp_config_t wasp_config = wasp_get_config();
  //struct wasp_handle wasp_hd = {0};
  wasp_handle_t* wasp_hd = malloc(sizeof(wasp_handle_t));
  ESP_ERROR_CHECK(wasp_init(wasp_hd, wasp_config));

  wasp_register_command_function(wasp_hd, &simple);
  //printf("Where's the problem here?\n");
  //simple_handler(&simple_frame);

  xTaskCreate(uart_event_task, "uart_event_task", 2048, wasp_hd, 12, NULL);
  ESP_LOGI(TAG, "UART task created.");
}
