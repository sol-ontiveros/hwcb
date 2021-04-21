

/*
Interrupt-based UART->parallel transceiver.
*/


#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"


const char* TAG = "Parallel Port";

//queue to for uart events
static QueueHandle_t uart_queue;

//UART-side configurations
#define PAR_TXD (CONFIG_PAR_UART_TXD)
#define PAR_RXD (CONFIG_PAR_UART_RXD)
#define PAR_RTS (UART_PIN_NO_CHANGE)
#define PAR_CTS (UART_PIN_NO_CHANGE)

#define PAR_UART_PORT_NUM      (CONFIG_PAR_UART_PORT_NUM)
#define PAR_UART_BAUD_RATE     (CONFIG_PAR_UART_BAUD_RATE)
#define PAR_TASK_STACK_SIZE    (CONFIG_PAR_TASK_STACK_SIZE)

#define BUF_SIZE (1024)


//parallel-side configurations
#define GPIO_OUTPUT_IO_0    12
#define GPIO_OUTPUT_IO_1    13
#define GPIO_OUTPUT_IO_2    14
#define GPIO_OUTPUT_IO_3    15
#define GPIO_OUTPUT_IO_4    16
#define GPIO_OUTPUT_IO_5    17
#define GPIO_OUTPUT_IO_6    18
#define GPIO_OUTPUT_IO_7    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3) | (1ULL<<GPIO_OUTPUT_IO_4) | (1ULL<<GPIO_OUTPUT_IO_5) | (1ULL<<GPIO_OUTPUT_IO_6) | (1ULL<<GPIO_OUTPUT_IO_7))


/*Control Signals*/
#define WREN             (CONFIG_PAR_WR_READY_IO_NUM)
#define WR_READY         (CONFIG_PAR_WREN_IO_NUM)

#ifdef CONFIG_PAR_USE_WREN
  #define PAR_USE_WREN 1
#else
  #define PAR_USE_WREN 0
#endif

#ifdef CONFIG_PAR_WAIR_FOR_WR_READY
  #define PAR_WAIT_FOR_WR_READY 1
#else
  #define PAR_WAIT_FOR_WR_READY 0
#endif

#define GPIO_INPUT_PIN_SEL  ((1ULL<<WR_READY) | (1ULL<<WREN))
#define ESP_INTR_FLAG_DEFAULT 0



//configure GPIO parallel port
void par_port_init(void) {
  ESP_LOGI(TAG, "Initializing parralel port.");
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 1;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}


/*For the sake of optimized write speeds, the following write functions are defined
based on configuration in the kconfig menu.*/
#if (PAR_USE_WREN && PAR_WAIT_FOR_WR_READY)
  void par_port_write(uint8_t byte) {
    while(!(gpio_get_level(WR_READY)); //wait for write ready to go high.
    REG_WRITE(GPIO_OUT_REG, byte << GPIO_OUTPUT_IO_0); //put data on port
    gpio_set_level(WREN, 1);
    //delay? Or is the FPGA fast enough?
    gpio_set_level(WREN, 0);
  }
#elif PAR_USE_WREN
  void par_port_write(uint8_t byte) {
    REG_WRITE(GPIO_OUT_REG, byte << GPIO_OUTPUT_IO_0); //put data on port
    gpio_set_level(WREN, 1);
    //delay? Or is the FPGA fast enough?
    gpio_set_level(WREN, 0);
  }
#else
  //write to the 8-bit parallel port
  void par_port_write(uint8_t byte) {
    REG_WRITE(GPIO_OUT_REG, byte << GPIO_OUTPUT_IO_0);
  }
#endif


static void parallel_uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, BUF_SIZE);
            ESP_LOGI(TAG, "UART[%d] event:", PAR_UART_PORT_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA EVT]: Started. Size: %d", event.size);
                    uart_read_bytes(PAR_UART_PORT_NUM,  dtmp, event.size, 100);

                    for (int i=0; i<event.size; i++) {
                      par_port_write(dtmp[i]);
                      //ESP_LOGI(TAG, "Wrote to parallel: %x", dtmp[i]);
                    }


                    ESP_LOGI(TAG, "[UART DATA EVT]: Complete");
                    //uart_write_bytes(PAR_UART_PORT_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(PAR_UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(PAR_UART_PORT_NUM);
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
                    uart_get_buffered_data_len(PAR_UART_PORT_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(PAR_UART_PORT_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    uart_flush_input(PAR_UART_PORT_NUM);
                    /*if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(PAR_UART_PORT_NUM);
                    } else {
                        uart_read_bytes(PAR_UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(PAR_UART_PORT_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
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


esp_err_t uart_parallel_port_init(void) {
  uart_config_t uart_config = {
      .baud_rate = PAR_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  if(!(uart_is_driver_installed(PAR_UART_PORT_NUM))) {
    ESP_ERROR_CHECK(uart_param_config(PAR_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(PAR_UART_PORT_NUM, PAR_TXD, PAR_RXD, PAR_RTS, PAR_CTS));
    ESP_ERROR_CHECK(uart_driver_install(PAR_UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));

    par_port_init();
    xTaskCreate(parallel_uart_event_task, "uart_parallel_task", PAR_TASK_STACK_SIZE, NULL, 12, NULL);

    ESP_LOGI(TAG, "Initialized parallel port - WREN: %i | Wait on WR READY: %i", PAR_USE_WREN, PAR_WAIT_FOR_WR_READY);
    return ESP_OK;
  }
  else {
    ESP_LOGE(TAG, "UART driver already installed on port %i", PAR_UART_PORT_NUM);
    return ESP_FAIL;
  }
}
