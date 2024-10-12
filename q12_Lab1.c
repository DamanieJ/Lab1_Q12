#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOSConfig.h"

#define UART_BUFFER_SIZE       1024
#define UART_LARGE_BUFFER_SIZE 2048
#define NO_DATA                0
#define QUEUE_WAIT_TIME_MS     100
#define ONE_SECOND_DELAY_MS    1000
#define HALF_SECOND_DELAY_MS   500
#define GPIO_HIGH              1
#define GPIO_LOW               0

/* Global queue handle */
QueueHandle_t message_queue_handle;

/* Message structure */
typedef struct {
    int command; /* 1 to turn LED on, 0 to turn LED off */
} Message;

/* Function to configure the UART */
void ConfigureUART(void)
{
    uart_config_t uart_config = {
        .baud_rate = 7,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config); /* Configure UART with specified settings */
    uart_driver_install(UART_NUM_0, UART_LARGE_BUFFER_SIZE, NO_DATA, NO_DATA, NULL, NO_DATA);
}

/* Function to configure GPIO */
void ConfigureGPIO(void)
{
    gpio_config_t gpio_settings = {
        .mode         = GPIO_MODE_DEF_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << GPIO_NUM_2) /* Configure GPIO2 */
    };
    gpio_config(&gpio_settings); /* Apply GPIO configuration */
}

/* Task 1: Send command to turn LED on */
void TaskSendOnCommand(void *pv_parameters)
{
    Message message;
    while (1)
    {
        message.command = 1; /* Command to turn LED on */
        if (xQueueSend(message_queue_handle, &message, (TickType_t) QUEUE_WAIT_TIME_MS) != pdPASS)
        {
            printf("Failed to send on command\n");
        }
        vTaskDelay(HALF_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Delay for 500ms */
    }
}

/* Task 2: Send command to turn LED off */
void TaskSendOffCommand(void *pv_parameters)
{
    Message message;
    while (1)
    {
        message.command = 0; /* Command to turn LED off */
        if (xQueueSend(message_queue_handle, &message, (TickType_t) QUEUE_WAIT_TIME_MS) != pdPASS)
        {
            printf("Failed to send off command\n");
        }
        vTaskDelay(ONE_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Delay for 1 second */
    }
}

/* Task 3: Send commands to turn LED on or off alternately */
void Task3(void *pv_parameters)
{
    Message message;
    while (1)
    {
        message.command = 1; /* Command to turn LED on */
        if (xQueueSend(message_queue_handle, &message, (TickType_t) QUEUE_WAIT_TIME_MS) != pdPASS)
        {
            printf("Failed to send on command\n");
        }
        vTaskDelay(HALF_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Delay for 500ms */

        message.command = 0; /* Command to turn LED off */
        if (xQueueSend(message_queue_handle, &message, (TickType_t) QUEUE_WAIT_TIME_MS) != pdPASS)
        {
            printf("Failed to send off command\n");
        }
        vTaskDelay(ONE_SECOND_DELAY_MS / portTICK_PERIOD_MS); /* Delay for 1 second */
    }
}

/* Main application entry point */
void app_main(void)
{
    ConfigureGPIO(); /* Configure GPIO */
    ConfigureUART(); /* Configure UART */

    /* Create a queue capable of holding 5 Message items */
    message_queue_handle = xQueueCreate(5, sizeof(Message));

    if (message_queue_handle != NULL)
    {
        /* Create tasks with different priorities */
        xTaskCreate(TaskSendOnCommand, "Task Send On Command", UART_LARGE_BUFFER_SIZE, NULL, 1, NULL);
        xTaskCreate(TaskSendOffCommand, "Task Send Off Command", UART_LARGE_BUFFER_SIZE, NULL, 2, NULL);
        xTaskCreate(Task3, "Task3", UART_LARGE_BUFFER_SIZE, NULL, 3, NULL);
    }
}