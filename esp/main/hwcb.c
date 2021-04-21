
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

//components
#include "wasp.h"
#include "parallel_port.h"


#ifdef CONFIG_WASP_ENABLE
  #define WASP_ENABLE 1
#else
  #define WASP_ENABLE 0
#endif

#ifdef CONFIG_PAR_PORT_ENABLE
  #define PAR_PORT_ENABLE 1
#else
  #define PAR_PORT_ENABLE 0
#endif

//tag for ESP logging macros.
static const char* TAG = "HWCB_MAIN";


/***********************************************************************************************************
****************************************** WASP COMMAND HANDLERS *******************************************
***********************************************************************************************************/

const char* FAKE_TAG = "Fake Tag";

esp_err_t echo_handler(wasp_handle_t* handle, wasp_frame_t* frame) {
  printf("This is a printf from the command handler function for code: %x\n", frame->cmd);
  ESP_LOGI(FAKE_TAG, "And this is a log info.");

  return wasp_transmit_frame(handle, frame);
}

const wasp_command_function_t echo = {
  .command = "Echo",
  .cmd_hex_code = 0x10,
  .handler = echo_handler
};

esp_err_t sum_handler(wasp_handle_t* handle, wasp_frame_t* frame) {
  printf("This is a printf from the command handler function for code: %x\n", frame->cmd);

  int32_t return_data = 0;
  int32_t* rx_data;
  if ((frame->data_num*sizeof(int32_t)) < handle->config.max_tx_buf) {
    rx_data = calloc(frame->data_num*sizeof(int32_t), sizeof(int32_t));
  }
  else{
    ESP_LOGW(FAKE_TAG, "Recieved frame was larger than max allowable data buffer.");
    return ESP_FAIL;
  }
  //copy all data to int32s.
  for (int i=0; i<frame->data_num; i++) {
    memcpy(&rx_data[i], &frame->data[i*frame->data_size], frame->data_size);
  }
  //sum all received data.
  for (int i=0; i<frame->data_num; i++) {
    return_data=return_data+rx_data[i];
  }

  if(handle->config.debug) {
    printf("Calc'd Sum: %i", return_data);
    printf("first data in buffer: %i", rx_data[0]);
  }

  free(rx_data);
  ESP_LOGI(FAKE_TAG, "Sum: %i", return_data);
  return wasp_send(handle, frame->cmd, 4, 1, &return_data);

  //return ESP_OK;
}

const wasp_command_function_t sum = {
  .command = "Sum",
  .cmd_hex_code = 0x11,
  .handler = sum_handler
};

/*************************************************************************************************************
**************************************************************************************************************/




/******************************************************************************
******************************** MAIN *****************************************
******************************************************************************/
void app_main(void)
{
  #if (WASP_ENABLE && PAR_PORT_ENABLE)
    #if CONFIG_WASP_UART_PORT==CONFIG_PAR_UART_PORT_NUM
      #error "WASP and UART->parallel enabled on the same port."
    #endif
  #elif WASP_ENABLE
      //setup wasp
      wasp_config_t wasp_config = wasp_get_config();
      wasp_handle_t* wasp_hd = wasp_init(&wasp_config);
      if(wasp_hd==NULL) {
        ESP_LOGE(TAG, "Failed to create WASP handle.");
      }

      ESP_ERROR_CHECK(wasp_register_command_function(wasp_hd, &echo));
      ESP_ERROR_CHECK(wasp_register_command_function(wasp_hd, &sum));

  #elif PAR_PORT_ENABLE
      par_port_init();

  #else
    ESP_LOGW(TAG, "No components enabled!");
  #endif
}
