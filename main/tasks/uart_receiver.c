#include "uart_receiver.h"

// -- Task definition --
void uart_receiver( void* pvParameters ){
    /*
        [INFO] Notification Indexes
        Index 0 source - uart_sender, whenever the request is sent  
    */
    uint8_t bufferRX[256];
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t another_request_needed = TRUE;

    while( true ) {
        // Reset the check
        another_request_needed = TRUE;

        // -- [BLOCKING] Wait for the notification --
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // -- Main logic --
        int len = uart_read_bytes(UART_PORT_NUM, bufferRX, sizeof(bufferRX), UART_DELAY_TIME/portTICK_PERIOD_MS);
        // ESP_LOGI(printerVar, "Length of data received from UART: %d bytes", len);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, bufferRX[i], &msg, &status)) {
                    // Got one complete MAVLink message
                    handle_mavlink_msg(&msg);

                    // Some information was received, handle_mavlink_msg will trigger next request
                    another_request_needed = FALSE;
                }
            }
        }

        // At this point, length == 0 OR the received message is not MAVlink-like.
        if( another_request_needed ){
            // Trigger another altitude requestUART_DEBUG_FLAG
            #if ( UART_DEBUG_FLAG )
                ESP_LOGI(printerVar, "Uart_receiver didt'get any MAVLINK msg.");
            #endif

            xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
        }
    }
}