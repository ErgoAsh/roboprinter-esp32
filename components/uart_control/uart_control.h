/*
 * uart_control.h
 *
 *  Created on: 31 may 2022
 *      Author: ErgoAsh
 */

#ifndef MAIN_UART_CONTROL_H_
#define MAIN_UART_CONTROL_H_

void uart_init(void);

void uart_event_task(void *pvParameters);

#endif /* MAIN_UART_CONTROL_H_ */
