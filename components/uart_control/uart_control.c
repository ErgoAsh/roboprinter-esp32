/*
 * uart_control.c
 *
 *  Created on: 31 may 2022
 *      Author: ErgoAsh
 */
#include "uart_control.h"
#include "servo_control.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

const char* UART_TAG = "RP_UART";

static const int BUFFER_SIZE = 1024;
static const int UART_PORT_NUMBER = UART_NUM_0;
static const char PATTERN_CHAR_NUMBER = 1;

static QueueHandle_t uart_queue;

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

	uart_param_config(UART_PORT_NUMBER, &uart_config);
    uart_set_pin(UART_PORT_NUMBER, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUMBER, BUFFER_SIZE * 2, BUFFER_SIZE * 2, 20, &uart_queue, 0);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr(UART_PORT_NUMBER, '.', PATTERN_CHAR_NUMBER, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_PORT_NUMBER, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUFFER_SIZE);

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart_queue, (void*) &event, (portTickType) portMAX_DELAY)) {
            bzero(dtmp, BUFFER_SIZE);
            ESP_LOGI(UART_TAG, "UART%d event occurred", UART_PORT_NUMBER);
            switch (event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(UART_TAG, "Data size: %d", event.size);
                    uart_read_bytes(UART_PORT_NUMBER, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(UART_TAG, "Content: %s", (const char*) dtmp);
                    //uart_write_bytes(UART_PORT_NUMBER, (const char*) dtmp, event.size);
                    ESP_LOGI(UART_TAG, "End");
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(UART_TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUMBER);
                    xQueueReset(UART_PORT_NUMBER);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(UART_TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUMBER);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(UART_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(UART_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(UART_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_PORT_NUMBER, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_PORT_NUMBER);
                    ESP_LOGI(UART_TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_PORT_NUMBER);
                    } else {
                        uart_read_bytes(UART_PORT_NUMBER, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHAR_NUMBER + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_PORT_NUMBER, pat, PATTERN_CHAR_NUMBER, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(UART_TAG, "read data: %s", dtmp);
                        ESP_LOGI(UART_TAG, "read pat : %s", pat);



                    }
                    break;
                //Others
                default:
                    ESP_LOGI(UART_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
