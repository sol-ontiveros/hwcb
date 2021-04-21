
#ifndef PARALLEL_PORT_H
#define PARALLEL_PORT_H


esp_err_t uart_parallel_port_init(void);

void par_port_init(void);

void par_port_write(uint8_t byte);

#endif
