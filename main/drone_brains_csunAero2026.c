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
#define UART_TX_PIN_NUM 43 // D6 on XIAO = TX
#define UART_RX_PIN_NUM 44 // D7 on XIAO = RX
#define UART_RTS_PIN_NUM 8 
#define UART_CTS_PIN_NUM 9
#define UART_PORT_NUM UART_NUM_2

#define MAVLINK_SYSTEM_ID 2
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER 

// --- Task handlers ---
static TaskHandle_t uart_sender_taskHandler = NULL;
static TaskHandle_t uart_receiver_taskHandler = NULL;
static const char* printerTask = "printer"; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal

// --- Helper functions ---
static void handle_mavlink_msg( mavlink_message_t* msg ){
    
    switch( msg->msgid ){
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
                // Declare and reset the global position data holder
                mavlink_global_position_int_t glob_pos_int_holder = {0};

                // Get all fields in payload (into global_position)
                // TODO: change logic to getting a single field
                mavlink_msg_global_position_int_decode(msg, &glob_pos_int_holder);

                // Print the resulting data
                ESP_LOGI( printerTask, "!---FINAL DATA---!\nRelative altitude: %ld", glob_pos_int_holder.relative_alt);
            }
            break;
        
        case MAVLINK_MSG_ID_HEARTBEAT:
            {
                ESP_LOGI(printerTask, "Heart beat received.");
            }
            break;
        
        default:
            {
                ESP_LOGI(printerTask, "[WARNING] Incorrect msgid: %d", msg->msgid);
            }
            break;
    }
}

// --- Task definitions ---
void uart_receiver( void* pvParameters ){
    uint8_t bufferRX[256];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, bufferRX, sizeof(bufferRX), pdMS_TO_TICKS(20));
        // ESP_LOGI(printerTask, "Length of data received from UART: %d bytes", len);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, bufferRX[i], &msg, &status)) {
                    // Got one complete MAVLink message
                    handle_mavlink_msg(&msg);
                }
            }
        }
    }
}


void uart_sender( void* pvParameters ){
    /* Debugging, to be converted into ISR-based approach ----------------------------------------*/
    uint8_t bufferTX[128];

    // Declare the msg struct and ensure it doesn't contain any memory noise
    mavlink_message_t msg = {0};
    // Generate the message for requesting data
    mavlink_msg_command_long_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, 1, 0, 512, 0, 33, 0, 0, 0, 0, 0, 0);

    // Pack the message to get the final MavLink data pocket to be transmitted over UART
    uint16_t mavlinkData_len = mavlink_msg_to_send_buffer(bufferTX, &msg);
    ESP_LOGI( printerTask, "Mavlink data has %d length", mavlinkData_len );

    // Write data to UART.
    uart_write_bytes(UART_PORT_NUM, bufferTX, mavlinkData_len);

    // Wait for the response to come in
    //vTaskDelay(10000/portTICK_PERIOD_MS);
    ESP_LOGI( printerTask, "Getting data from uart..." );

    // Read data from UART.
    // TODO: turn the logic into "read byte ->  parse byte". Curent logic: "read all -> parse all byte by byte" is inefficient.
    uint8_t bufferRX[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
    length = uart_read_bytes(UART_PORT_NUM, bufferRX, length, 100);
    
    /* End of debugging ------------------------------------------------------------------------- */

    while( true ){ 
        // endless loop
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    // Optional UART clearing
    // uart_driver_delete( uart_num );
}


// --- Main function ---
// TODO: Implement log file for the error handling
// TODO: Implement sleep modes
void app_main(void) {

    // TODO: move the init logic to a separate task
    // -- UART init begin --
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
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK( uart_param_config(UART_PORT_NUM, &uart_config) );

    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK( uart_set_pin(UART_PORT_NUM, UART_TX_PIN_NUM, UART_RX_PIN_NUM, UART_RTS_PIN_NUM, UART_CTS_PIN_NUM) );

    // Setup UART in rs485 half duplex mode
    ESP_ERROR_CHECK( uart_set_mode(UART_PORT_NUM, UART_MODE_UART) );
 
    // Enable interrupts
    //ESP_ERROR_CHECK( uart_enable_intr_mask(TO BE FINISHED...) );

    // -- UART init end --

    // A task for reseiving data over UART(using Mavlink2 as the payload format)
    BaseType_t status = xTaskCreatePinnedToCore(
        uart_receiver, // function
        "UART_RECEIVER", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        1, // Priority
        &uart_receiver_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );
    
    if(status != pdPASS){
        ESP_LOGE( printerTask, "Failed to create the UART_RECEIVER task" );
    }
    
    /*    
    // A task for requesting data over UART(using Mavlink2 as the payload format)
    BaseType_t status = xTaskCreatePinnedToCore(
        uart_sender, // function
        "UART_SENDER", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        1, // Priority
        &uart_sender_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );
    
    if(status != pdPASS){
        ESP_LOGE( printerTask, "Failed to create the UART_SENDER task" );
    }
    */
}
