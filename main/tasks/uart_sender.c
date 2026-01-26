#include "uart_sender.h"

// -- Task definition --
void uart_sender( void* pvParameters ){
    /*
        [INFO] - UART gets enabled on boot via app_main,
        is re-enabled when no message or not MAVlink-like message is received via uart_receiver,
        is disabled when the state = STATE_LANDED via handle_mavlink_msg,
        is re-enabled when during the STATE_LANDED->STATE_LOW_ALT transition via the gate_controller.

        Notification index 0 sources: app_main uart_receiver, gate_controller, handle_mavlink_msg.
    */

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
       
        // Flush the RX buffer to get the latest data
        // TODO: Potentially link up first, then flush RX once and then start sending requests
        //uart_flush_input(UART_PORT_NUM);
        /*
            TODO: subscribe, do not poll
            uart_receiver 
                runs all the time unless the state is STATE_LANDED
                follows pre-specified frequency

            uart_sender only:
                waits until it sees HEARTBEAT (learn sysid/compid)
                sends SET_MESSAGE_INTERVAL
                then it can stop
        */

        // Write data to UART.
        ESP_LOGI( printerVar, "Requesting data from mavlink..." );
        uart_write_bytes(UART_PORT_NUM, bufferTX, mavlinkData_len);

        // Allow the read 
        xTaskNotifyGiveIndexed(uart_receiver_taskHandler, 0);
        
        // Stay idle for a bit to decrease the computational cost
        vTaskDelay(UART_DELAY_TIME/portTICK_PERIOD_MS);
    }

    // Optional UART clearing
    // uart_driver_delete( uart_num );
}