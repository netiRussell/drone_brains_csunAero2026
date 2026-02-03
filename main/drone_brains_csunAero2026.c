/*
[WARNING] - This code requires multiple notification array entries.
    To turn it on:
    In menuconfig → Component config → FreeRTOS → Task notifications
    Set “Number of task notification array entries” (or similarly named) to 2 or more.
    That sets configTASK_NOTIFICATION_ARRAY_ENTRIES >= 2.
*/

// -- Includes -- 
#include "drone_brains_csunAero2026.h"
#include "tasks/ir_controller.h"
#include "tasks/gate_controller.h"
#include "tasks/uart_sender.h"
#include "tasks/uart_receiver.h"


// -- Global variable for debugging -- 
#if (PWM_GATE_REC_DEBUG_FLAG)
    static int debug_only_counter = 0;
#endif


// --- Main function ---
void app_main(void) {
    // -- ISR based, GPIO init --
    // The GPIO will wait for the payload to close an open circuit to trigger the raise of the pick-up mechanism
    
    // GPIO config
    gpio_config_t gpios_config = {0};
    gpios_config.mode = GPIO_MODE_INPUT;
    gpios_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpios_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpios_config.intr_type = GPIO_INTR_POSEDGE;

    // Gate GPIO for the payload receiving
    gpios_config.pin_bit_mask = 1ULL << GATE_GPIO_NUM_REC;
    ESP_ERROR_CHECK( gpio_config(&gpios_config) );

    // Configure ISRs
    ESP_ERROR_CHECK( gpio_install_isr_service(ESP_INTR_FLAG_IRAM) );
    ESP_ERROR_CHECK( gpio_isr_handler_add(GATE_GPIO_NUM_REC, ir_rec_handler, NULL) );

    // Stop interrupts from this pin until the gate is lowered 
    gpio_intr_disable(GATE_GPIO_NUM_REC);
    
    // For debugging, comment out all tasks initializations in app_main except this GPIO init
    #if( PWM_GATE_REC_DEBUG_FLAG )
        // Enable interrupt
        gpio_intr_enable(GATE_GPIO_NUM_REC);
        ESP_LOGI(printerVar, "[WARNING] PWM GATE GPIO Interrupt debug mode has been enabled");

        while(TRUE){
            ESP_LOGI(printerVar, "Interrupt count: %d", debug_only_counter);

            // Give some time for a treat to the Watch Dog
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    #endif
    

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
    
    // -- Queues init --
    xQueueIRdata = xQueueCreate(1, sizeof(ir_nec_scan_code_t));


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
        ESP_LOGE( printerVar, "Failed to create the GATE_CONTROLLER task" );
        abort();
    }

    // A task for managing the IR
    status = xTaskCreatePinnedToCore(
        ir_controller, // function
        "IR_CONTROLLER", // name
        4096, // stack size in bytes
        NULL, // pvParameters
        3, // Priority
        &ir_taskHandler, // Handler to reffer to the task
        1 // Core ID
    );
    
    if(status != pdPASS){
        ESP_LOGE( printerVar, "Failed to create the IR_CONTROLLER task" );
        abort();
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
        ESP_LOGE( printerVar, "Failed to create the UART_RECEIVER task" );
        abort();
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
        ESP_LOGE( printerVar, "Failed to create the UART_SENDER task" );
        abort();
    }

    
    // Enable UART_sender
    xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
}
