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
#include <rmw_microros/rmw_microros.h>
 
void servo_control_init(void);
 
void data_received_callback(const double* angle_array);
 
double bytes_to_double(uint8_t* bytes, bool big_endian);
double radians_to_degrees(const double position_radians);
uint16_t angle_to_pulse(const double angle, const uint8_t servo);
 
bool linreg(uint8_t n, const double x[], const double y[], double* a, double* b);
 
#endif /* MAIN_SERVO_CONTROL_H_ */
