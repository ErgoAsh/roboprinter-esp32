/*
 * servo_control.c
 *
 *  Created on: 30 may 2022
 *      Author: ErgoAsh
 */
#include "servo_control.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "endian.h"
#include "driver/mcpwm.h"

const char* SERVO_TAG = "RP_SERVO";

static float servo_0_angle = 0;
static float servo_1_angle = 0;
static float servo_2_angle = 0;
static float servo_3_angle = 0;

static const float servo_us_data[4][3] = {
		{ 2450, 1525, 525 },
		{ 2475, 1575, 625 },
		{ 625, 1525, 2475 },
		{ 525, 1500, 2475 }
};
static const float servo_angle_data[4][3] = {
		{ -90, 0, 90 },
		{ -90, 0, 90 },
		{ -90, 0, 90 },
		{ -90, 0, 90 }
};

static float slope[4] = {0, 0, 0, 0};
static float intersection[4] = {0, 0, 0, 0};

void data_received_callback(const uint8_t* data_buffer) {
	float angle_array[4];

	// Iterate over 4 servos
	for (size_t i = 0; i < 20; i += 4) {
		uint32_t number = 0;

		for (size_t j = i; j < (i + 4); j++) {
			// Shift existing number by 8 bits and append one byte using OR
			// operation (since bits does not overlap, nothing will be
			// overwritten)
			number = (number << 8) | data_buffer[j];

			//for (size_t mask = (1 << CHAR_BIT); mask >>= 1;) {
				//ESP_LOGI(UART_TAG, "read data: %s", data_buffer);
				//Serial.print((data_buffer[j] & mask)? '1' : '0');
			//}
			//Serial.print(" ");
		}
		//Serial.println();

		bswap32(number); // Swap endian

		float angle;
		memcpy(&angle, &number, sizeof(number));
		angle_array[i / 4] = angle;
	}

	// TODO calculate everything in radians
	servo_0_angle = radians_to_degrees(angle_array[0]);
	servo_1_angle = radians_to_degrees(angle_array[1]);
	servo_2_angle = radians_to_degrees(angle_array[2]);
	servo_3_angle = radians_to_degrees(angle_array[3]);

	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, angle_to_pulse(servo_0_angle, 0)); // Servo0
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, angle_to_pulse(servo_1_angle, 1)); // Servo1
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, angle_to_pulse(servo_2_angle, 2)); // Servo2
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, angle_to_pulse(servo_3_angle, 3)); // Servo3
}

float radians_to_degrees(const float position_radians) {
	return position_radians * 57.2958; // 180 / 1*pi [deg/rad]
}

uint16_t angle_to_pulse(const float angle, const uint8_t servo) {
	return round(slope[servo] * angle + intersection[servo]);
}

bool linreg(uint8_t n, const float x[], const float y[], float* a, float* b) {
	float sumx = 0.0;                      /* sum of x     */
	float sumx2 = 0.0;                     /* sum of x**2  */
	float sumxy = 0.0;                     /* sum of x * y */
	float sumy = 0.0;                      /* sum of y     */
	float sumy2 = 0.0;                     /* sum of y**2  */

    for (size_t i = 0; i < n; i++){
        sumx  += x[i];
        sumx2 += sqrt(x[i]);
        sumxy += x[i] * y[i];
        sumy  += y[i];
        sumy2 += sqrt(y[i]);
    }

    float denom = (n * sumx2 - sqrt(sumx));
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *a = 0;
        *b = 0;
        return false;
    }

    *a = (n * sumxy  -  sumx * sumy) / denom;
    *b = (sumy * sumx2  -  sumx * sumxy) / denom;

    return true;
}

void servo_control_init(void) {
	for (size_t i = 0; i < 4; i++) {
		if (!linreg(3, servo_angle_data[i], servo_us_data[i], &slope[i], &intersection[i])) {
			ESP_LOGE(SERVO_TAG, "Linear regression has returned an error");
		} else {
			ESP_LOGI(SERVO_TAG, "Linear regression: a=%f, b=%f", slope[i], intersection[i]);
		}
	}
}

