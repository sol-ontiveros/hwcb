
#ifndef WASP_H
#define WASP_H

#include <stdio.h>
#include "driver/uart.h"


#define WASP_ACK 0xF0000000
#define WASP_NACK 0xFF000000
#define WASP_RESP_ERR 0xFE000000


typedef struct wasp_config {
  uart_port_t uart_port;
  int max_rx_buf;
  int max_tx_buf;
  int max_cmds;
  int rx_timeout;
  int debug;
} wasp_config_t;

typedef struct wasp_frame {
  int32_t msg_len; //the total message length
  int32_t cmd; //the command byte of the frame
  int32_t data_num; //the number of elements in data[]
  int32_t data_size; //the size of 1 element in data[]
  uint8_t checksum; //CRC8, little-endian
  uint8_t frame_valid; //flag to indiccate if problems were encountered in frame rx/creation.
  uint8_t checksum_pass; //true if checksum has passed wasp_crc_check().
  uint8_t data[]; //the frame data
} wasp_frame_t;


//TODO: The handle needs to be of type wasp_handle_t, but I can't think of how to do that right now.
typedef struct wasp_command_function {
  const char* command;
  int cmd_hex_code; //hex code that will be received over UART
  //parame handle MUST be of type wasp_handle_t
  esp_err_t (*handler)(void* handle, wasp_frame_t *frame);
} wasp_command_function_t;

typedef struct wasp_handle {
  wasp_config_t config;
  wasp_command_function_t** commands;
} wasp_handle_t;


int wasp_crc_check(wasp_frame_t* frame_ptr);

wasp_frame_t* wasp_receive(wasp_handle_t* handle);

esp_err_t wasp_transmit_frame(wasp_handle_t* handle, wasp_frame_t* frame);

esp_err_t wasp_send(wasp_handle_t* handle, uint32_t command, uint32_t data_member_size, uint32_t num_data_members, void* data);

wasp_handle_t* wasp_init(wasp_config_t* wasp_config);

wasp_config_t wasp_get_config();

int wasp_find_command(wasp_handle_t* handle, int32_t command);

esp_err_t wasp_register_command_function(wasp_handle_t* handle, const wasp_command_function_t* cmd_function);

esp_err_t wasp_rx_routine(wasp_handle_t* handle);

static void wasp_uart_event_task(void *pvParameters);

#endif
