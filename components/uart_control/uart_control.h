/*
 * uart_control.h
 *
 *  Created on: 31 may 2022
 *      Author: ErgoAsh
 */

#ifndef MAIN_UART_CONTROL_H_
#define MAIN_UART_CONTROL_H_

#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <rmw_microros/rmw_microros.h>

bool esp32_serial_open(struct uxrCustomTransport * transport);
bool esp32_serial_close(struct uxrCustomTransport * transport);
size_t esp32_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t esp32_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void subscription_callback(const void* msgin);
void uart_init(void);

#endif /* MAIN_UART_CONTROL_H_ */
