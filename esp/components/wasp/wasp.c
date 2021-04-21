
/*
|\     /|(  ___  )(  ____ \(  ____ )
| )   ( || (   ) || (    \/| (    )|
| | _ | || (___) || (_____ | (____)|
| |( )| ||  ___  |(_____  )|  _____)
| || || || (   ) |      ) || (
| () () || )   ( |/\____) || )
(_______)|/     \|\_______)|/


***** Widely-Applicable Serial Protocol ******
General-use protocol for data/command transfers.


WASP frame format:

Frame length: int
Command: int
Number of items in data: int
Size of items in data: int
Data: (Number of items specified of size specified)
CRC-8 checksum: unsigned char


*/


#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>

#include "wasp.h"
#include "uart.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp32/rom/crc.h"


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


#ifdef CONFIG_WASP_DEBUG
  #define WASP_DBG CONFIG_WASP_DEBUG
#else
  #define WASP_DBG 0
#endif
//the size of the wasp frame header in 8-bit bytes
#define WASP_HEADER_SIZE 16


static const char *TAG = "WASP";




 uint8_t wasp_crc_create(wasp_frame_t* frame) {
  if(!frame->frame_valid) {
    ESP_LOGW(TAG, "CRC check on invalid frame - results may not be accurate.");
  }

  //create a buffer that excludes the rx'd checksum.
  uint32_t buffer_size = (WASP_HEADER_SIZE + (frame->data_num*frame->data_size));
  uint8_t* buf = calloc(buffer_size, sizeof(uint8_t));

  if(buf==NULL) {
    free(buf);
    ESP_LOGE(TAG, "Unable to allocate data buffer for CRC.");
    return -1;
  }

  //copy frame header to buffer.
  memcpy(&buf[0*sizeof(uint32_t)], &frame->msg_len, sizeof(frame->msg_len));
  memcpy(&buf[1*sizeof(uint32_t)], &frame->cmd, sizeof(frame->cmd));
  memcpy(&buf[2*sizeof(uint32_t)], &frame->data_num, sizeof(frame->data_num));
  memcpy(&buf[3*sizeof(uint32_t)], &frame->data_size, sizeof(frame->data_size));

  //insert data offset by size of frame header
  for (int i = 0; i<(buffer_size-WASP_HEADER_SIZE); i++) {
    buf[i+(WASP_HEADER_SIZE)] = frame->data[i];
  }

  uint8_t crc_val = crc8_le(0, buf, buffer_size);

  //print some debug messages.
  if (WASP_DBG) {
    printf("Calculated CRC: %x\n", crc_val);
    printf("CRC data buffer: ");
    for (int i=0; i<buffer_size; i++) {
      printf("%x\n", buf[i]);
    }
    printf("\n");

    printf("crc buffer size: %i\n", buffer_size);
  }

  free(buf);
  return ~crc_val; //ESP api generates inverted CRC.
}



//generate a checksum and check it against the stored value int he frame_ptr.
int wasp_crc_check(wasp_frame_t* frame) {
  uint8_t crc_val = wasp_crc_create(frame);

  //print some debug messages.
  if (WASP_DBG) {
    printf("RX'd CRC: %x\n", frame->checksum);
  }

  if (frame->checksum == crc_val) {
    return 1;
  }
  return 0;
}



wasp_frame_t* wasp_receive(wasp_handle_t* handle) {
  ESP_LOGI(TAG, "Beginning data RX.");
  /*if(handle->config.debug) {
    printf("handle in receive: %i \n%i \n%i \n", handle->config.max_rx_buf, handle->config.max_tx_buf, handle->config.uart_port);
  }*/
  if(uart_is_driver_installed(handle->config.uart_port)) {
    //TODO: check for ACK, NACK, and ERR? Should be handled by wasp_write.

    //get frame header - total frame size and data length info.
    uint32_t* frame_header = (uint32_t*) calloc(4, sizeof(uint32_t));
    if(frame_header==NULL) {
      free(frame_header);
      ESP_LOGE(TAG, "Unable to allocate frame header memory.");
      return NULL;
    }
    uart_read_bytes(handle->config.uart_port,  frame_header, (4*sizeof(uint32_t)), handle->config.rx_timeout);

    //Give the buffer time to receive the full frame if necessary. I don't want to break anything
    //here if frame is not complete, but I do want to print a warning.
    uint8_t frame_valid_flag = 1;
    size_t buffered_data = 0;
    int timeout_counter = handle->config.rx_timeout;
    uart_get_buffered_data_len(handle->config.uart_port, &buffered_data);
    while(timeout_counter>0) {
      if (!(buffered_data >= (frame_header[0]-(sizeof(*frame_header)*sizeof(frame_header[0]))))) {
        //vTaskDelayUntil(&xLastWakeTime, xDelayFreq);
        uart_get_buffered_data_len(handle->config.uart_port, &buffered_data);
      }
      else if (buffered_data >= (frame_header[0]-(sizeof(*frame_header)*sizeof(frame_header[0])))) {
        break;
      }
      timeout_counter-=1;
      if(timeout_counter==0) {
        ESP_LOGW(TAG, "Frame received is smaller than indicated. May have received incomplete frame.");
        frame_valid_flag = 0; //set flag so we know to check for corrupt data if we want to.
      }
    }

    //get data based on received data lengths
    uint8_t* data_buf;
    uint8_t rx_chksum = 0;
    if((frame_header[2]*frame_header[3]) < (uint32_t) handle->config.max_rx_buf) {
      //create space for data received, plus checksum byte.
      data_buf = calloc(frame_header[2]*frame_header[3], sizeof(uint8_t));
      if (data_buf==NULL) {
        free(frame_header);
        free(data_buf);
        ESP_LOGE(TAG, "Unable to allocate data buffer memory.");
        return NULL;
      }
      uart_read_bytes(handle->config.uart_port,  data_buf, frame_header[2]*frame_header[3], handle->config.rx_timeout);
      uart_read_bytes(handle->config.uart_port,  &rx_chksum, 1, handle->config.rx_timeout);
    }
    else {
      ESP_LOGW(TAG, "Received data size is larger than max allowed buffer size.");
      free(frame_header);
      return NULL;
    }

    //pack data into wasp frame struct.
    wasp_frame_t* ret_frame = malloc(sizeof(*ret_frame) + (frame_header[2]*frame_header[3]) + 1);
    if (ret_frame==NULL) {
      ESP_LOGW(TAG, "Couldn't allocate memory for frame data");
      free(ret_frame);
      return NULL;
    }

    //Assign rx'd frame header values to the frame struct.
    ret_frame->msg_len =      frame_header[0];
    ret_frame->cmd =          frame_header[1];
    ret_frame->data_num =     frame_header[2];
    ret_frame->data_size =    frame_header[3];
    ret_frame->checksum =     rx_chksum;
    ret_frame->frame_valid =  frame_valid_flag;

    //Copy all data from rx buf to the frame.
    for (int i=0; i<frame_header[2]*frame_header[3]; i++) {
      ret_frame -> data[i] = data_buf[i];
    }

    //Done with the buffers now.
    free(frame_header);
    free(data_buf);
    ESP_LOGI(TAG, "Frame received: Frame size: %i bytes, Command code: %x", ret_frame->msg_len, ret_frame->cmd);

    //perform crc validation on received frame.
    ret_frame->checksum_pass=0;
    if (wasp_crc_check(ret_frame)) {
      ESP_LOGI(TAG, "Verified CRC");
      ret_frame->checksum_pass = 1;
    }
    else {
      ESP_LOGW(TAG, "Checksum failed. RX'd: %i, Calc'd: %i", ret_frame->checksum, wasp_crc_create(ret_frame));
    }

    //flush input after rx. This could discard other sent data, so will need to handle
    //in the ISR w/ ack/nack system.
    uart_flush_input(handle->config.uart_port);
    return ret_frame;
  }

  //return an error if the UART driver is not installed.
  ESP_LOGE(TAG, "[PORT%i]: UART driver is not installed. Cannot send or receive data.", handle->config.uart_port);
  return NULL;
}


//parse and send a (COMPLETE) wasp frame over the specified UART.
//Use wasp_send below to construct & transmit the data frame.
esp_err_t wasp_transmit_frame(wasp_handle_t* handle, wasp_frame_t* frame) {
  //verify driver is installed on the port we are trying to write to.
  if(uart_is_driver_installed(handle->config.uart_port)) {
    if (frame->checksum==NULL) {
      ESP_LOGW(TAG, "[TX] Frame pointer did not contain checksum. Generating one now.");
      uint8_t crc_val = wasp_crc_create(frame);
      frame->checksum = crc_val;
      frame->checksum_pass = NULL; //This is more for receive. Set to NULL just in case.
    }

    //Buffer = size of frame header + data field + one checksum byte.
    uint32_t buffer_size = (4*sizeof(uint32_t) + (frame->data_num*frame->data_size) + 1);
    if(buffer_size>handle->config.max_tx_buf) {
      ESP_LOGE(TAG, "[TX] Transmit data is larger than max allowed buffer size.");
      return ESP_FAIL;
    }

    uint8_t* buf = malloc(buffer_size);
    if(buf==NULL) {
      free(buf);
      ESP_LOGE(TAG, "[TX] Unable to allocate data buffer.");
      return -1;
    }

    //move struct members to the appropriate place in the buffer.
    memcpy(&buf[0*sizeof(uint32_t)], &frame->msg_len, sizeof(frame->msg_len));
    memcpy(&buf[1*sizeof(uint32_t)], &frame->cmd, sizeof(frame->cmd));
    memcpy(&buf[2*sizeof(uint32_t)], &frame->data_num, sizeof(frame->data_num));
    memcpy(&buf[3*sizeof(uint32_t)], &frame->data_size, sizeof(frame->data_size));
    //move all data, offset by the size of the frame header.
    for (int i = 0; i<(frame->data_num*frame->data_size); i++) {
      buf[i+WASP_HEADER_SIZE] = frame->data[i];
    }
    //insert checksum into last position in transmit buffer.
    buf[buffer_size-1] = frame->checksum;

    if(handle->config.debug) {
      printf("buffer bytes from wasp_transmit: \n");
      for(int i = 0; i<buffer_size; i++) {
        printf("%x\n", buf[i]);
      }
      //printf("\n");
      //printf("As const char: %s\n", (const char*) buf);
    }


    //Send data and wait for transmit to complete.
    int data_sent = uart_write_bytes(handle->config.uart_port, (const char*) buf, buffer_size);
    if (data_sent!=buffer_size) {
      ESP_LOGE(TAG, "[TX] Problem loading data into ring buffer. Data size: %i, Data loaded: %i", buffer_size, data_sent);
      free(buf);
      return ESP_FAIL;
    }

    free(buf);
    /*if(handle->config.debug) {
      ESP_ERROR_CHECK(uart_wait_tx_done(handle->config.uart_port, 100));
    }*/
    ESP_LOGI(TAG, "Transmitted %i bytes.", data_sent);
    return ESP_OK;
  }
  //return an error if the UART driver is not installed.
  ESP_LOGE(TAG, "[PORT%i]: UART driver is not installed. Cannot send or receive data.", handle->config.uart_port);
  return ESP_FAIL;
}



/* Construct a frame and transmit it based on the given parameters.
(Data_member_size is the size of each data member in data[], to be used by the receiving node. data[] is then interpreted as an 8-bit integer for transmitting across the UART.) */
esp_err_t wasp_send(wasp_handle_t* handle, uint32_t command, uint32_t data_member_size, uint32_t num_data_members, void* data) {
  uint32_t data_total_size = (data_member_size*num_data_members);
  if(data_total_size>handle->config.max_tx_buf) {
    ESP_LOGE(TAG, "[TX] Data to send is larger than max allowed buffer size (data size: %i, buffer size: %i)", data_total_size, handle->config.max_tx_buf);
    return ESP_FAIL;
  }

  uint32_t message_length = (WASP_HEADER_SIZE + data_total_size +1); //the length of the transmitted message will be header+data+1 checksum byte.

  wasp_frame_t* tx_frame = malloc(sizeof(*tx_frame) + data_total_size);

  //construct the wasp data frame.
  tx_frame->msg_len =    message_length;
  tx_frame->cmd =        command;
  tx_frame->data_num =   num_data_members;
  tx_frame->data_size =  data_member_size;

  //copy data into frame to transmit.
  uint8_t* data_buf = calloc(data_total_size, sizeof(uint8_t));
  if (data_buf==NULL) {
    ESP_LOGE(TAG, "[TX] Unable to allocate buffer.");
    free(data_buf);
    free(tx_frame);
    return ESP_FAIL;
  }
  memcpy(data_buf, data, data_total_size);
  for(int i=0; i<data_total_size; i++) {
    tx_frame->data[i] = data_buf[i];
  }

  //tx_frame.data = *data_buf;
  //tx_frame.data = calloc(data_total_size, sizeof(uint8_t));
  //memcpy(tx_frame.data, data, data_total_size);

  tx_frame->frame_valid = 1;

  //uint8_t crc_val = wasp_crc_create(&tx_frame);
  //tx_frame.checksum = crc_val;
  tx_frame->checksum = wasp_crc_create(tx_frame);
  tx_frame->checksum_pass = wasp_crc_check(tx_frame);

  ESP_LOGI(TAG, "[PORT%i] Data to send. (Size: %i).", ((int) handle->config.uart_port), message_length);

  if (handle->config.debug) {
    printf("Data size: %i\n", data_total_size);
    printf("Data buf build by wasp_send:\n");
    for (int i = 0; i<data_total_size; i++) {
      printf("%x\n", data_buf[i]);
    }
    printf("Wasp send, from tx_frame:\n");
    for (int i = 0; i<data_total_size; i++) {
      printf("%x\n", tx_frame->data[i]);
    }
  }

  esp_err_t ret_val = wasp_transmit_frame(handle, tx_frame);
  //TODO: this needs to move up once this is working.
  free (data_buf);
  free(tx_frame);
  return ret_val;
}


//Used to indicate data was received, but that it was rejected or invalid.
esp_err_t wasp_nack(wasp_handle_t* handle) {
  char nack_msg = WASP_NACK;
  int data_sent = uart_write_bytes(handle->config.uart_port, &nack_msg, 1);
  ESP_LOGW(TAG, "[NACK]");
  uart_flush(handle->config.uart_port);
  return ESP_OK;
}

//Used to indicate data was received, but that it was rejected or invalid.
esp_err_t wasp_ack(wasp_handle_t* handle) {
  char nack_msg = WASP_ACK;
  int data_sent = uart_write_bytes(handle->config.uart_port, &nack_msg, 1);
  ESP_LOGI(TAG, "[ACK]");
  return ESP_OK;
}


wasp_handle_t* wasp_init(wasp_config_t* wasp_config) {

  /*if (wasp_config==NULL) {
    ESP_LOGI(TAG, "No configuration specified. Using default config.");
    wasp_config_t tmp_config = wasp_get_config();
    wasp_config_t* wasp_config = &tmp_config;
  }*/
  if(wasp_config==NULL) {
    ESP_LOGE(TAG, "Received invalid conifguration.");
  }
  wasp_handle_t* wasp_hd = malloc(sizeof(wasp_handle_t));
  //ESP_ERROR_CHECK(wasp_init(wasp_hd, wasp_config));
  //printf("Where's the problem here?\n");
  //simple_handler(&simple_frame);

  ESP_LOGI(TAG, "Beginning initialization.");
  //TODO: Add some config validation here, if needed.

  /*************** UART INIT and CONFIG *************************************
  ****************************************************************************/
  //TODO: This doesn't belong here.
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
  /*******************************************************************************
  *******************************************************************************/

  //Set uart pattern detect function.
  //uart_enable_pattern_det_baud_intr(CONFIG_SW_CTRL_UART_PORT, '+', PATTERN_CHR_NUM, 9, 0, 0);
  //Reset the pattern queue length to record at most 20 pattern positions.
  //uart_pattern_queue_reset(CONFIG_SW_CTRL_UART_PORT, 20);

  ESP_LOGI(TAG, "Sofware control port initialized.");

  wasp_hd -> config = *wasp_config;
  wasp_hd -> commands = calloc(wasp_config->max_cmds, sizeof(wasp_command_function_t *));
  if (!wasp_hd -> commands) {
      ESP_LOGE(TAG, "Failed to allocate memory for WASP command function handlers.");
      free(wasp_hd);
      return NULL;
  }

  ESP_ERROR_CHECK(uart_param_config(sw_ctrl_port, &uart_config));
  if (USE_HW_FLOWCTRL) {
    uart_set_hw_flow_ctrl(sw_ctrl_port, UART_HW_FLOWCTRL_CTS_RTS, DEFAULT_FLOWCTRL_THRESH);
  }
  ESP_ERROR_CHECK(uart_set_pin(sw_ctrl_port, CONFIG_UART_TXD, CONFIG_UART_RXD, UART_RTS, UART_CTS));
  ESP_ERROR_CHECK(uart_driver_install(sw_ctrl_port, CONFIG_UART_BUFFER_SIZE * 2, CONFIG_UART_BUFFER_SIZE * 2, 20, &uart_queue, 0));
  xTaskCreate(wasp_uart_event_task, "uart_event_task", CONFIG_UART_TASK_STACK_SIZE, wasp_hd, 12, NULL);
  ESP_LOGI(TAG, "WASP UART task created.");
  //Flush uart input on init - make sure that the next received frame will be spaced correctly.
  uart_flush(wasp_hd->config.uart_port);
  ESP_LOGI(TAG, "Initialization complete. UART Port: %i, debug: %i", wasp_hd->config.uart_port, wasp_hd->config.debug);

  return wasp_hd;
}


wasp_config_t wasp_get_config()  {
  wasp_config_t ret_config = {
    .uart_port = CONFIG_WASP_UART_PORT,
    .max_tx_buf = CONFIG_WASP_MAX_TX_BUF,
    .max_rx_buf = CONFIG_WASP_MAX_RX_BUF,
    .max_cmds = CONFIG_WASP_MAX_CMDS,
    .rx_timeout = CONFIG_WASP_RX_TIMEOUT,
    .debug = WASP_DBG
  };

  return ret_config;
}


//Check for the specified command function in the current handle.
int wasp_find_command(wasp_handle_t* handle, int32_t command) {
  for (int i=0; i<handle->config.max_cmds; i++) {
    if (handle->commands[i]->cmd_hex_code == command) {
      return i;
    }
  }
  return -1;
}


esp_err_t wasp_register_command_function(wasp_handle_t* handle, const wasp_command_function_t* cmd_function) {
  if (cmd_function==NULL) {
    ESP_LOGE(TAG, "Invalid command function struct.");
    return ESP_ERR_INVALID_ARG;
  }


  for (int i=0; i < handle->config.max_cmds; i++) {

    //register next available command function
    if (handle->commands[i] == NULL) {
      handle->commands[i] = malloc(sizeof(wasp_command_function_t));
      if (handle->commands[i] == NULL) {
        free(handle->commands[i]);
        ESP_LOGE(TAG, "Failed to allocate memory for command handler: %s", cmd_function->command);
        return ESP_FAIL;
      }
      //copy struct members over.
      handle->commands[i]->command =      cmd_function->command;
      handle->commands[i]->cmd_hex_code = cmd_function->cmd_hex_code;
      handle->commands[i]->handler =      cmd_function->handler;

      ESP_LOGI(TAG, "Registered command handler %i - Name: %s, Command: 0x%x", i, cmd_function->command, cmd_function->cmd_hex_code);
      return ESP_OK;
    }

    //Check if command function is already registered.
    //since command functions are entered at the next available position, this should run on all registered commands.
    if(handle->commands[i]->cmd_hex_code == cmd_function->cmd_hex_code) {
      ESP_LOGW(TAG, "Selected command %x is already registered at position %i under name %s", handle->commands[i]->cmd_hex_code, i, handle->commands[i]->command);
      return ESP_FAIL;
    }
  }

  //Should only get here if all available command function slots are full.
  ESP_LOGE(TAG, "Unable to register command function %s", cmd_function->command);
  return ESP_FAIL;
}


//This function finds the command from the received frame and calls the appropriate handler function.
//It should be called by the receive ISR.
esp_err_t wasp_call_command_handler(wasp_handle_t* handle, wasp_frame_t* frame) {
  int idx = wasp_find_command(handle, frame->cmd);
  if (idx>=0) {
    ESP_LOGI(TAG, "Calling command function %s, (hex cmd: %x)", handle->commands[idx]->command, handle->commands[idx]->cmd_hex_code);
    return handle->commands[idx]->handler(handle, frame);
  }
  ESP_LOGW(TAG, "No valid command function registered for command %x.", frame->cmd);
  return ESP_FAIL;
}


//This will run instead of the real routine when debug is set in the config menu.
esp_err_t wasp_debug_isr(wasp_handle_t* handle, wasp_frame_t* frame) {
  //wasp_transmit_frame(rx_frame);
  //uint32_t debug_data[4] = {1024, 1025, 1026, 1027};
  //uint32_t debug_data[4] = {1,2,3,4};
  //uint32_t debug_data[4] = {0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
  //uint8_t debug_data[5] = {1, 2, 3, 4, 5};


  //printf("calculate another crc: ");
  //uint8_t buffer[28] = {0x1d,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x03,0x00,0x00,0x00};
  //printf("%x\n", ((uint8_t) ~crc8_le(0, &buffer, 28)));

  return wasp_call_command_handler(handle, frame);
  //return wasp_send(handle, 0xAA, 4, 4, &debug_data);
  //return wasp_send(handle, 0xFF, 1, 5, (uint8_t*) &debug_data);
  //return ESP_OK
}


/* This is the main UART receive ISR. It should receive and parse the frame to
a wasp_handle_t, perform the necessary checks, and call the appropriate command
function/handler based on the received command and data. */
esp_err_t wasp_rx_routine(wasp_handle_t* handle) {
  wasp_frame_t* rx_frame = wasp_receive(handle);
  //wasp_receive returns null if there were problems. free frame and respond with NACK so we can retry data receive.
  if(rx_frame==NULL) {
    free(rx_frame);
    ESP_LOGW(TAG, "Problem receiving frame. Command will not be run.");
    if(handle->config.debug) {
      return ESP_OK;
    }
    return wasp_nack(handle);
  }

  //continue with handling if frame is valid and passed checksum test.
  if(rx_frame->frame_valid && rx_frame->checksum_pass) {
    //a little space for testing stuff-write it in function wasp_debug_isr.
    if(handle->config.debug) {
      ESP_LOGW(TAG, "Debug set to TRUE... running test rx routine.");
      printf("Test routine begun.\n");
      esp_err_t ret_val = wasp_debug_isr(handle, rx_frame);
      printf("End of test routine.\n");
      free(rx_frame);
      return ret_val;
    }

    //place the real code here.
    return wasp_call_command_handler(handle, rx_frame);
    free(rx_frame);
    //return wasp_ack(handle);
    return ESP_OK;
  }

  ESP_LOGE(TAG, "Frame was not valid.");
  //return wasp_nack(handle);
  return ESP_FAIL;
}




/************************************************************************************************
UART interrupt handler setup. This should call the wasp rx/parse routines on data receipt,
and report all other UART errors.

register with xTaskCreate().
**************************************************************************************************/
static void wasp_uart_event_task(void *pvParameters)
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
                    uart_flush_input(wasp_handle->config.uart_port);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(wasp_handle->config.uart_port);
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
