#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <c_library_v2/common/mavlink.h>

// Macros
#define UART_TX_PIN_NUM 6
#define UART_RX_PIN_NUM 7
#define UART_RTS_PIN_NUM 8
#define UART_CTS_PIN_NUM 9
#define UART_PORT_NUM 2

#define MAVLINK_SYSTEM_ID 2
#define MAVLINK_COMPONENT_ID 191 

// --- Task handlers ---
static TaskHandle_t uart_communicator_taskHandler = NULL;
static const char* printerTask = "printer"; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal

// --- Task definitions ---
void uart_communicator( void* pvParameters ){

    // Setup UART buffered IO with event queue
    const uint16_t uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK( uart_driver_install(UART_PORT_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0) );

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK( uart_param_config(UART_PORT_NUM, &uart_config) );

    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK( uart_set_pin(UART_PORT_NUM, UART_TX_PIN_NUM, UART_RX_PIN_NUM, UART_RTS_PIN_NUM, UART_CTS_PIN_NUM) );

    // Setup UART in rs485 half duplex mode
    ESP_ERROR_CHECK( uart_set_mode(UART_PORT_NUM, UART_MODE_UART) );

    /* Debugging, to be converted into ISR-based approach ----------------------------------------*/
    uint8_t buffer[128];

    // Generate the message for requesting data
    mavlink_message_t* msg = {};        
    mavlink_msg_command_long_pack(MAVLINK_SYSTEM_ID, MAV_COMP_ID_ONBOARD_COMPUTER, msg, 1, 0, 512, 0, 33, 0, 0, 0, 0, 0, 0);

    // Pack the message to get the final MavLink data pocket to be transmitted over UART
    uint16_t mavlinkData_len = mavlink_msg_to_send_buffer(buffer, msg);
    ESP_LOGI( printerTask, "Mavlink data has %d length", mavlinkData_len );
    ESP_LOGI( printerTask, "DEBUG: %d", MAV_COMP_ID_ONBOARD_COMPUTER );

    // Write data to UART.
    uart_write_bytes(UART_PORT_NUM, buffer, sizeof(buffer) / sizeof(uint8_t));

    // Wait for the response to come in
    vTaskDelay(10000/portTICK_PERIOD_MS);
    ESP_LOGI( printerTask, "Getting data from uart..." );

    // Read data from UART.
    uint8_t data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
    length = uart_read_bytes(UART_PORT_NUM, data, length, 100);

    for(uint8_t i = 0; i < 128; i++){
        ESP_LOGI( printerTask, "Data %d: %d", i, data[i] );
    }

    // Flush the RX FIFO buffer
    uart_flush(UART_PORT_NUM);

    /* End of debugging ------------------------------------------------------------------------- */

    // Enable interrupts
    //ESP_ERROR_CHECK( uart_enable_intr_mask(TO BE FINISHED...) );

    while( true ){ 
        // endless loop
    }

    // Optional UART clearing
    // uart_driver_delete( uart_num );
}


// --- Main function ---
// TODO: Implement log file for the error handling
// TODO: Implement sleep modes
void app_main(void) {

    // A task for the UART communication(over Mavlink2)
    BaseType_t status = xTaskCreatePinnedToCore(
        uart_communicator, // function
        "UART_COMMUNICATOR", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        1, // Priority
        &uart_communicator_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );

    if(status != pdPASS){
        ESP_LOGE( printerTask, "Failed to create the UART_COMMUNICATOR task" );
    }
}
