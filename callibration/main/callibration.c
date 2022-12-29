#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "endian.h"
#include "argtable3/argtable3.h"
#include "linenoise/linenoise.h"

#define PROMPT_STR CONFIG_IDF_TARGET
static const char* TAG = "callibration";
const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

static int pwm_output_pin = GPIO_NUM_26;
static int pwm_frequency = 50;
static int pwm_duty_us = 1500;
static bool is_pwm_enabled = false;
static bool is_debugging_enabled = false;

static int set_pin(int argc, char **argv)
{
    pwm_output_pin = atoi(argv[1]);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pwm_output_pin); // TODO test is valid
    return ESP_OK;
}

static int set_frequency(int argc, char **argv)
{ 
    int frequency = atoi(argv[1]);

    if (frequency  > 3000) {
        printf("Requested frequency is too high!");
        return ESP_ERR_INVALID_ARG;
    }

    if (frequency < 100) {
        printf("Requested frequency is too low!");
        return ESP_ERR_INVALID_ARG;
    }

    pwm_frequency = frequency;
    mcpwm_config_t pwm_config = {
        .frequency = pwm_frequency, // digital servos: 333 Hz, analog servos: 50 Hz
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER, // Count up to max
        .duty_mode = MCPWM_DUTY_MODE_0 // Non-inverted signal
    };

    // PWM channels: 0A, 0B
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    return ESP_OK;
}

static int set_duty(int argc, char **argv)
{
    pwm_duty_us = atoi(argv[1]);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, pwm_duty_us);
    return ESP_OK;
}

static int toggle_pwm(int argc, char **argv)
{
    if (is_pwm_enabled)
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    else 
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

    is_pwm_enabled = !is_pwm_enabled;
    return ESP_OK;
}

static int set_debugging(int argc, char **argv)
{
    char* arg = argv[1];
    while (strcmp(arg, "true"))
    {
        is_debugging_enabled = true;
    }

    while (strcmp(arg, "false"))
    {
        is_debugging_enabled = false;
    }
    // TODO stop/start debugging timer
    return ESP_OK;
}

static void register_commands(void)
{
    const esp_console_cmd_t set_pin_cmd = {
        .command = "pin",
        .help = "Set PWM output pin",
        .hint = NULL,
        .func = &set_pin,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_pin_cmd));

    const esp_console_cmd_t set_frequency_cmd = {
        .command = "pin",
        .help = "Set PWM timer fequency",
        .hint = NULL,
        .func = &set_frequency,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_frequency_cmd));

    const esp_console_cmd_t set_duty_cmd = {
        .command = "pin",
        .help = "Set pulse width in us",
        .hint = NULL,
        .func = &set_duty,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_duty_cmd));

    const esp_console_cmd_t set_debugging_cmd = {
        .command = "pin",
        .help = "Set debugging mode (true/false)",
        .hint = NULL,
        .func = &set_debugging,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_debugging_cmd));

    const esp_console_cmd_t toggle_pwm_cmd = {
        .command = "pin",
        .help = "Toggle PWM output - enabled or disabled",
        .hint = NULL,
        .func = &toggle_pwm,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&toggle_pwm_cmd));
}

static void initialize_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
#if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
#elif SOC_UART_SUPPORT_XTAL_CLK
        .source_clk = UART_SCLK_XTAL,
#endif
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                256, 0, 0, NULL, 0) );
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
        .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

    /* Set command maximum length */
    linenoiseSetMaxLineLen(console_config.max_cmdline_length);

    /* Don't return empty lines */
    linenoiseAllowEmpty(false);
}

void register_prompt(void)
{
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */

    printf("\n"
            "This is an example of ESP-IDF console component.\n"
            "Type 'help' to get the list of commands.\n"
            "Use UP/DOWN arrows to navigate through command history.\n"
            "Press TAB when typing command name to auto-complete.\n"
            "Press Enter or Ctrl+C will terminate the console environment.\n");

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
                "Your terminal application does not support escape sequences.\n"
                "Line editing and history features are disabled.\n"
                "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = PROMPT_STR "> ";
#endif //CONFIG_LOG_COLORS
    }

}

void check_console_loop(void)
{
    /* Get a line using linenoise.
     * The line is returned when ENTER is pressed.
     */
    char* line = linenoise(prompt);
    if (line == NULL) { /* Break on EOF or error */
        return;
    }
    /* Add the command to the history if not empty*/
    if (strlen(line) > 0) {
        linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
        /* Save command history to filesystem */
        linenoiseHistorySave(HISTORY_PATH);
#endif
    }

    /* Try to run the command */
    int ret;
    esp_err_t err = esp_console_run(line, &ret);
    if (err == ESP_ERR_NOT_FOUND) {
        printf("Unrecognized command\n");
    } else if (err == ESP_ERR_INVALID_ARG) {
        // command was empty
    } else if (err == ESP_OK && ret != ESP_OK) {
        printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
    } else if (err != ESP_OK) {
        printf("Internal error: %s\n", esp_err_to_name(err));
    }
    /* linenoise allocates line buffer on the heap, so need to free it */
    linenoiseFree(line);

}

void app_main(void)
{
    initialize_console();
    register_commands();
    esp_console_register_help_command();

    register_prompt();
    while(true) {
        check_console_loop();
    }

    ESP_LOGE(TAG, "Error or end-of-input, terminating console");
    esp_console_deinit();

}
