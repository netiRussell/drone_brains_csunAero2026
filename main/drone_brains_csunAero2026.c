/*
[WARNING] - This code requires multiple notification array entries.
    To turn it on:
    In menuconfig → Component config → FreeRTOS → Task notifications
    Set “Number of task notification array entries” (or similarly named) to 2 or more.
    That sets configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2.
*/
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
#include "driver/rmt_tx.h"
#include <c_library_v2/common/mavlink.h>


// -- Macros --
#define UART_TX_PIN_NUM 43 // D6 on XIAO = TX
#define UART_RX_PIN_NUM 44 // D7 on XIAO = RX
#define UART_RTS_PIN_NUM 8 
#define UART_CTS_PIN_NUM 9
#define UART_PORT_NUM UART_NUM_2

#define MAVLINK_SYSTEM_ID 2
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER 

// ALT_GATE_THRESHOLD is always < ALT_AIRBORNE_THRESHOLD
#define ALT_AIRBORNE_THRESHOLD 800
#define ALT_GATE_THRESHOLD -100 

#define PWM_GATE_GPIO_NUM_SIG   5 // Sends the PWM signal, D4
#define PWM_GATE_GPIO_NUM_REC   6 // Gets triggered when the payload is inside the pick-up mechanism, D5
#define PWM_GATE_TIMER_ID       LEDC_TIMER_0
#define PWM_GATE_TIMER_RES      LEDC_TIMER_12_BIT
#define PWM_GATE_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define PWM_GATE_CHANNEL        LEDC_CHANNEL_0

#define RMT_GPIO_NUM    1 // TODO: figure out if D3 is suitable, and what is its GPIO number


// -- States and state functions ( To be modified only with Mutexes/Spinlocks ) --
typedef enum {
    STATE_AIRBORNE,
    STATE_LOW_ALT,
    STATE_LANDED
} states_t;
states_t state = STATE_LOW_ALT;

static portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;
static states_t check_state(){
    return state;
}

static void update_state_to(states_t new_state){
    taskENTER_CRITICAL(&state_spinlock);

    // [CRITICAL SECTION]
    state = new_state;

    taskEXIT_CRITICAL(&state_spinlock);
}


// --- Task handlers ---
static TaskHandle_t uart_sender_taskHandler = NULL;
static TaskHandle_t uart_receiver_taskHandler = NULL;
static TaskHandle_t gate_taskHandler = NULL;
static TaskHandle_t ir_taskHandler = NULL;
static const char* printerTask = "printer"; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal


// --- ISRs ---
static void IRAM_ATTR gate_rec_handler(void* arg){
    // Notify the pick-up mechanism(gate) that it start rising now
    vTaskGenericNotifyGiveFromISR(gate_taskHandler, 1, NULL);

    // Stop more interrupts from this pin until the gate is lowered again
    gpio_intr_disable(PWM_GATE_GPIO_NUM_REC);
}


// --- Helper functions ---
static void handle_mavlink_msg( mavlink_message_t* msg ){
    
    switch( msg->msgid ){
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            /* 
            // To be utilized if more info is needed: 
            // Declare and reset the global position data holder
            mavlink_global_position_int_t glob_pos_int_holder = {0};
            
            // Get all fields in payload (into global_position)
            // TODO: change logic to getting a single field
            mavlink_msg_global_position_int_decode(msg, &glob_pos_int_holder);
            */
            
            // Get altitude
            int32_t rel_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
            // Print the data
            ESP_LOGI( printerTask, "[DATA] Relative altitude: %ld", rel_alt);
            
            // State logic
            state = check_state();
            
            switch(state){
                case STATE_AIRBORNE: {
                    // Above ALT_AIRBORNE_THRESHOLD
                    // Drone is airborne and has reached a significant altitude 

                    // Check if landed
                    if( rel_alt < ALT_GATE_THRESHOLD ){
                        ESP_LOGI(printerTask, "[!STATE CHANGE!] STATE_AIRBORNE->STATE_LANDED");

                        // Change the state to STATE_LANDED
                        update_state_to(STATE_LANDED);

                        // Notify the gate controller to lower the gate
                        xTaskNotifyGiveIndexed(gate_taskHandler, 0);
                    }
                }
                break;

                case STATE_LOW_ALT: {
                    // Below ALT_AIRBORNE_THRESHOLD
                    // Drone has recently or is about to take off

                    // Check if above certain altitude (ALT_AIRBORNE_THRESHOLD)
                    if( rel_alt > ALT_AIRBORNE_THRESHOLD ){
                        ESP_LOGI(printerTask, "[!STATE CHANGE!] STATE_LOW_ALT->STATE_AIRBORNE");

                        // Change the state to STATE_AIRBORNE
                        update_state_to(STATE_AIRBORNE);
                    }

                }
                break;

                case STATE_LANDED: {
                    // Below ALT_GATE_THRESHOLD(which is < ALT_AIRBORNE_THRESHOLD)
                    // Drone has landed and will pick-up the payload

                    /*
                        !Notice, the altitude request logic is disabled in this state
                        
                        This state is updated to STATE_LOW_ALT in the gate_controller 
                        along with the semaphore update that allows UART_sender to 
                        continue sending altitude requests.

                        Enabling UART_sender signifies: drone is about to take off
                    */

                    // Exit the function early
                    return;
                }
                break;

                default: {}
                break;
            }


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

    // Allow UART_sender to send another request                        
    xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
}


// --- Task definitions ---
void ir_controller( void* pvParameters ){
    /*
        [INFO] Notification Indexes
        Index 0 source - gate_controller, whenever the gate is lowered 
    */

    // -- Init --
    // TX channel init
    rmt_channel_handle_t tx_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
        .gpio_num = RMT_GPIO_NUM,         // GPIO number
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256 Bytes
        .resolution_hz = 1 * 1000 * 1000, // 1 MHz tick resolution, i.e., 1 tick = 1 µs
        .trans_queue_depth = 4,           // set the number of transactions that can pend in the background
        .flags.invert_out = false,        // do not invert output signal
        .flags.with_dma = false,          // do not need DMA backend
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan));


    // -- Main logic --
    while( true ){
        // -- [BLOCKING] Wait for the notification --
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
        ESP_LOGI(printerTask, "IR has been triggered");

        // While the payload hasn't arrivied, keep sending the NEC signal
        // TODO: implement shared with gate_controller resource
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}


void gate_controller( void* pvParameters ){
    /*
        [INFO] Notification Indexes
        Index 0 source - handle_mavlink_msg, whenever the drone has landed
        Index 1 source - gate_rec_handler, ISR triggered by the payload
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
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // -- Lower the gate --
        ESP_LOGI( printerTask, "Gate controller has been triggered... duty = %ld", duty_2ms );

        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_2ms) ); // Set the new PWM with 2000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

        // Wait for the gate to fully extend down
        vTaskDelay(2000/portTICK_PERIOD_MS);

        // Reset and Put the peripheral back to sleep
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );
        
        // Send the corresponding IR command
        // xTaskNotifyGiveIndexed(ir_taskHandler, 0);

        // Enable the pin interrupt
        gpio_intr_enable(PWM_GATE_GPIO_NUM_REC);


        // -- [BLOCKING] Wait for the trigger to ensure the payload is inside --
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);
        //vTaskDelay(2000/portTICK_PERIOD_MS); // TO BE DELETED


        // -- Raise the gate --
        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_1ms) ); // Set the new PWM with 1000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

        // Wait for the gate to fully retract itself
        vTaskDelay(2000/portTICK_PERIOD_MS); 

        // Reset and Put the peripheral back to sleep 
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );

        // Change the state to STATE_LOW_ALT
        update_state_to(STATE_LOW_ALT);
        
        // Allow UART_sender to send another request                        
        xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
    }
}


void uart_receiver( void* pvParameters ){
    /*
        [INFO] Notification Indexes
        Index 0 source - uart_sender, whenever the request is sent  
    */
    uint8_t bufferRX[256];
    mavlink_message_t msg;
    mavlink_status_t status;

    while( true ) {
        // -- [BLOCKING] Wait for the notification --
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // -- Main logic --
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
    /*
        [INFO] - UART gets enabled on boot via app_main,
        is disabled when the state = STATE_LANDED via handle_mavlink_msg,
        is re-enabled when during the STATE_LANDED->STATE_LOW_ALT transition via the gate_controller.

        Notification index 0 sources: app_main, gate_controller, handle_mavlink_msg.
    */

    /* Debugging, to be converted into ISR-based approach ----------------------------------------*/
    uint8_t bufferTX[128];
    
    // Declare the msg struct and ensure it doesn't contain any memory noise
    mavlink_message_t msg = {0};
    // Generate the message for requesting data
    mavlink_msg_command_long_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, 1, 0, 512, 0, 33, 0, 0, 0, 0, 0, 0);
    
    // Pack the message to get the final MavLink data pocket to be transmitted over UART
    uint16_t mavlinkData_len = mavlink_msg_to_send_buffer(bufferTX, &msg);
    
    while( true ){ 
        // [BLOCKING] - Wait until you're allowed to send an altitude request
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // Write data to UART.
        uart_write_bytes(UART_PORT_NUM, bufferTX, mavlinkData_len);
        ESP_LOGI( printerTask, "Requesting data from mavlink..." );

        // Allow the read 
        xTaskNotifyGiveIndexed(uart_receiver_taskHandler, 0);
        
        // Stay idle for a bit to decrease the computational cost
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    /* End of debugging ------------------------------------------------------------------------- */

    // Optional UART clearing
    // uart_driver_delete( uart_num );
}


// --- Main function ---
// TODO: In UART_sender, first ensure that there is a connection, then start the request/handle mavlink logic
// TODO: Use multiple cores
void app_main(void) {

    // -- ISR based, GPIO init --
    // The GPIO will wait for the payload to close an open circuit to trigger the raise of the pick-up mechanism
    
    // GPIO config
    gpio_config_t pwm_gate_gpio_config = {0};
    pwm_gate_gpio_config.pin_bit_mask = 1ULL << PWM_GATE_GPIO_NUM_REC;
    pwm_gate_gpio_config.mode = GPIO_MODE_INPUT;
    pwm_gate_gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    pwm_gate_gpio_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pwm_gate_gpio_config.intr_type = GPIO_INTR_HIGH_LEVEL;
    ESP_ERROR_CHECK( gpio_config(&pwm_gate_gpio_config) );

    // Configure ISR
    ESP_ERROR_CHECK( gpio_install_isr_service(0) );
    ESP_ERROR_CHECK( gpio_isr_handler_add(PWM_GATE_GPIO_NUM_REC, gate_rec_handler, NULL) );

    // Stop interrupts from this pin until the gate is lowered 
    gpio_intr_disable(PWM_GATE_GPIO_NUM_REC);

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


    // -- RTOS tasks declarations --
    // A task for managing the pick-up mechanism
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


    // A task for managing the IR
    /* status = xTaskCreatePinnedToCore(
        ir_controller, // function
        "IR_CONTROLLER", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        2, // Priority
        &ir_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );
    
    if(status != pdPASS){
        ESP_LOGE( printerTask, "Failed to create the UART_SENDER task" );
    } */


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
    
    // Enable UART_sender(This is where the entire logic starts)
    xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
}
