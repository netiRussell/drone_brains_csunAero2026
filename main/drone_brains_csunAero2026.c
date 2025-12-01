#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <c_library_v2/common/mavlink.h>

// Macros
#define UART_TX_PIN_NUM 43 // D6 on XIAO = TX
#define UART_RX_PIN_NUM 44 // D7 on XIAO = RX
#define UART_RTS_PIN_NUM 8 
#define UART_CTS_PIN_NUM 9
#define UART_PORT_NUM UART_NUM_2

#define MAVLINK_SYSTEM_ID 2
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER 

#define ALT_GATE_THRESHOLD -500

#define PWM_GATE_GPIO_NUM_SIG   6 // Sends the PWM signal, D5
#define PWM_GATE_GPIO_NUM_REC   5 // Gets triggered when the payload is inside the pick-up mechanism, D4
#define PWM_GATE_TIMER_ID       LEDC_TIMER_0
#define PWM_GATE_TIMER_RES      LEDC_TIMER_12_BIT
#define PWM_GATE_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define PWM_GATE_CHANNEL        LEDC_CHANNEL_0

// --- Task handlers ---
static TaskHandle_t uart_sender_taskHandler = NULL;
static TaskHandle_t uart_receiver_taskHandler = NULL;
static TaskHandle_t gate_taskHandler = NULL;
static const char* printerTask = "printer"; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal

// --- Helper functions ---
static void handle_mavlink_msg( mavlink_message_t* msg ){
    
    switch( msg->msgid ){
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
                /* // To be utilied if more info is needed: 
                    // Declare and reset the global position data holder
                    mavlink_global_position_int_t glob_pos_int_holder = {0};
                    
                    // Get all fields in payload (into global_position)
                    // TODO: change logic to getting a single field
                    mavlink_msg_global_position_int_decode(msg, &glob_pos_int_holder);
                */

                uint32_t rel_alt = mavlink_msg_global_position_int_get_relative_alt(msg);

                if( rel_alt < ALT_GATE_THRESHOLD ){
                    // Notify the gate controller to lower the gate
                    xTaskNotifyGiveIndexed(gate_taskHandler, 0);
                }

                // Print the resulting data
                ESP_LOGI( printerTask, "[DATA] Relative altitude: %ld", rel_alt);
            }
            break;
        /*
        case MAVLINK_MSG_ID_HEARTBEAT:
            {
                ESP_LOGI(printerTask, "Heart beat received.");
            }
            break;
        */
        
        default:
            {
                // ESP_LOGI(printerTask, "[WARNING] Incorrect msgid: %d", msg->msgid);
            }
            break;
    }
}

// --- Task definitions ---
void gate_controller( void* pvParameters ){
    /*
        [Info] Notification Indexes
        Index 0 source - handle_mavlink_msg, whenever the drone has landed
        Index 1 source - _, ISR triggered by the payload
    */

    // --- Init the gate controller ---
    // Timer Init
    ledc_timer_config_t timer_config = {0};
    timer_config.speed_mode = PWM_GATE_SPEED_MODE;
    timer_config.duty_resolution = PWM_GATE_TIMER_RES;
    timer_config.timer_num = PWM_GATE_TIMER_ID;
    timer_config.freq_hz = 50; // Standard servo frequency. Period = 20000 micro sec
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK( ledc_timer_config(&timer_config) );

    // Channel Init
    ledc_channel_config_t channel_config = {0};
    channel_config.gpio_num = PWM_GATE_GPIO_NUM_SIG; 
    channel_config.speed_mode = PWM_GATE_SPEED_MODE;
    channel_config.channel = PWM_GATE_CHANNEL;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = PWM_GATE_TIMER_ID; 
    channel_config.duty = 0; // To be set later in the code
    channel_config.hpoint = 0; // Start the HIGH output in the beginning of each cycle
    channel_config.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;

    ESP_ERROR_CHECK( ledc_channel_config(&channel_config) );

    // Put the PWM peripheral to sleep with LOW idle level
    ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );

    // Precompute duties
    // Formula: number of max ticks in the clock * percentage of the clock that will stay High
    const uint32_t max_duty   = (1 << PWM_GATE_TIMER_RES) - 1; // Max value the clock reaches
    // Integer division is avoided by changing the formula into:
    const uint32_t duty_2ms   = (max_duty * 2000) / 20000; // full down
    const uint32_t duty_1ms   = (max_duty * 1000) / 20000; // full up
    const uint32_t duty_idle  = (max_duty * 1500) / 20000; // neutral

    // --- Main logic ---
    while( true ){
        // -- [BLOCKING] Wait for the notification --
        ulTaskNotifyTakeIndexed(0, 0, portMAX_DELAY);

        // -- Lower the gate --
        ESP_LOGI( printerTask, "Gate controller has been triggered..." );

        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_2ms) ); // Set the new PWM with 2000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_LOGI( printerTask, "!!!! 2000" );

        // Wait for the gate to fully extend down
        vTaskDelay(2000/portTICK_PERIOD_MS);
        
        // Reset and Put the peripheral back to sleep
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );


        // -- Wait for the trigger to ensure the payload is inside --
        // ulTaskNotifyTakeIndexed(1, 0, portMAX_DELAY);
        vTaskDelay(2000/portTICK_PERIOD_MS); // TO BE DELETED


        // -- Raise the gate --
        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_LOGI( printerTask, "!!!! 1000" );
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_1ms) ); // Set the new PWM with 1000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

        // Wait for the gate to fully retract itself
        vTaskDelay(2000/portTICK_PERIOD_MS); 

        // Reset and Put the peripheral back to sleep 
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );
    }
}

void uart_receiver( void* pvParameters ){
    uint8_t bufferRX[256];
    mavlink_message_t msg;
    mavlink_status_t status;

    while( true ) {
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
    
    while( true ){ 
        // Write data to UART.
        uart_write_bytes(UART_PORT_NUM, bufferTX, mavlinkData_len);

        ESP_LOGI( printerTask, "Getting data from uart..." ); 

        // Wait for the response to come in
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    /* End of debugging ------------------------------------------------------------------------- */

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


    // -- RTOS tasks declarations --
    // A task for requesting data over UART(using Mavlink2 as the payload format)
    BaseType_t status = xTaskCreatePinnedToCore(
        gate_controller, // function
        "GATE_CONTROLLER", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        2, // Priority
        &gate_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );
    
    if(status != pdPASS){
        ESP_LOGE( printerTask, "Failed to create the UART_SENDER task" );
    }

    // A task for reseiving data over UART(using Mavlink2 as the payload format)
    status = xTaskCreatePinnedToCore(
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
    
    // A task for requesting data over UART(using Mavlink2 as the payload format)
    status = xTaskCreatePinnedToCore(
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

    
}
