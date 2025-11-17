#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// --- Task handlers ---
static TaskHandle_t uart_communicator_taskHandler = NULL;

// --- Task definitions ---
void uart_communicator( void* pvParameters ){

    while( true ){ 
        // endless loop
    }
}


// --- Main function ---
// TODO: Implement log file for the error handling
void app_main(void) {

    // A task for the UART communication(over Mavlink2)
    ESP_ERROR_CHECK(
        xTaskCreatePinnedToCore(
            uart_communicator, // function
            "UART_COMMUNICATOR", // name
            4096, // stack size in bytes
            NULL, // pvParameters
            1, // Priority
            &uart_communicator_taskHandler, // Handler to reffer to the task
            1 // Core ID
        )
    );
}
