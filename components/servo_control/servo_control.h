/*
 * servo_control.h
 *
 *  Created on: 30 may 2022
 *      Author: ErgoAsh
 */

#ifndef MAIN_SERVO_CONTROL_H_
#define MAIN_SERVO_CONTROL_H_

#include <stdio.h>
#include <stdbool.h>

void servo_control_init(void);

float bytes_to_float(uint8_t* bytes, bool big_endian);

void data_received_callback(const uint8_t* data_buffer);

float radians_to_degrees(const float position_radians);

uint16_t angle_to_pulse(const float angle, const uint8_t servo);

bool linreg(uint8_t n, const float x[], const float y[], float* a, float* b);

#endif /* MAIN_SERVO_CONTROL_H_ */
