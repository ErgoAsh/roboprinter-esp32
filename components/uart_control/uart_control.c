/*
 * uart_control.c
 *
 *  Created on: 31 may 2022
 *      Author: ErgoAsh
 */
#include "uart_control.h"
#include "servo_control.h"

#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#define _BSD_SOURCE
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

const char* UART_TAG = "RP_UART";

//static const int BUFFER_SIZE = 1024;
static const int UART_PORT_NUMBER = UART_NUM_0;
static const int UART_BUFFER_SIZE = 512;

rcl_subscription_t subscriber;
sensor_msgs__msg__JointState recv_msg;

bool serial_open(struct uxrCustomTransport * transport) {
    //size_t * uart_port = (size_t*) transport->args;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(UART_PORT_NUMBER, &uart_config) == ESP_FAIL) {
        return false;
    }
    if (uart_set_pin(UART_PORT_NUMBER, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_FAIL) {
        return false;
    }
    if (uart_driver_install(UART_PORT_NUMBER, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;
}

bool serial_close(struct uxrCustomTransport * transport)
{
    return uart_driver_delete(UART_PORT_NUMBER) == ESP_OK;
}

size_t serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    const int txBytes = uart_write_bytes(UART_PORT_NUMBER, (const char*) buf, len);
    return txBytes;
}

size_t serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
    const int rxBytes = uart_read_bytes(UART_PORT_NUMBER, buf, len, timeout / portTICK_RATE_MS);
    return rxBytes;
}

void subscription_callback(const void* msgin)
{
	const sensor_msgs__msg__JointState* msg = (const sensor_msgs__msg__JointState*) msgin;
	printf("Received: %f\n", msg->position.data[0]);
    data_received_callback(msg->position.data);
}

void uart_event_task(void *pvParameters)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    //rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	//RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    //rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
    //RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "roboprinter_esp_rclc", "", &support));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"joint_states"));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

void uart_init(void) {
	rmw_uros_set_custom_transport(
		true,
		(void *) UART_PORT_NUMBER,
		serial_open,
		serial_close,
		serial_write,
		serial_read
	);

    xTaskCreate(uart_event_task,
       "uart_event_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL
    );
}
