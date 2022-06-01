/*
 * servo_control.h
 *
 *  Created on: 30 may 2022
 *      Author: ErgoA
 */
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/timer.h"
#include "driver/uart.h"

#include "servo_control.h"
#include "uart_control.h"

static const uint32_t PIN_BLINK = GPIO_NUM_5;
static const uint32_t PIN_SERVO_0 = GPIO_NUM_36;
static const uint32_t PIN_SERVO_1 = GPIO_NUM_39;
static const uint32_t PIN_SERVO_2 = GPIO_NUM_32;
static const uint32_t PIN_SERVO_3 = GPIO_NUM_33;

const char* TAG = "RP";

void init_status_diode(void) {
	gpio_reset_pin(PIN_BLINK);
	gpio_set_direction(PIN_BLINK, GPIO_MODE_OUTPUT);

//  TODO add timer
//	timer_config_t timer_config = {
//
//	}
//
//	timer_init(TIMER_GROUP_0, TIMER_0, timer_config);
}

void init_servo_communication(void) {
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO_0);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_SERVO_1);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_SERVO_2);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_SERVO_3);

	mcpwm_config_t pwm_config = {
			.frequency = 333, // digital servos: 333 Hz, analog servos: 50 Hz
			.cmpr_a = 0,
			.cmpr_b = 0,
			.counter_mode = MCPWM_UP_COUNTER, // Count up to max
			.duty_mode = MCPWM_DUTY_MODE_0 // Non-inverted signal
	};

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // 0A, 0B
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); // 1A, 1B
}

void init_uart_communication(void) {
	uart_init();
}

void app_main(void)
{
	init_status_diode();
	init_servo_communication();
	init_uart_communication();

	gpio_set_level(PIN_BLINK, true);
}
