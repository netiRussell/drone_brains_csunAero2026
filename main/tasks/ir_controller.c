#include "ir_controller.h"


// -- ISRs --
bool IRAM_ATTR ir_trans_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx){
    BaseType_t high_task_wakeup = pdFALSE;

    // Notify the IR task that it has finished current NEC transmission.
    vTaskNotifyGiveIndexedFromISR(ir_taskHandler, 0, &high_task_wakeup);

    return high_task_wakeup;
}

// -- Task definition --
void ir_controller( void* pvParameters ){
    /*
        [INFO] This task waits on a queue that passes pointer to the struct with NEC address and command
        Queue source: gate_controller, when the drone has landed and now it executed the drop-off and pick-up procedures

        [INFO] Notification Indexes
        Index 0 source - ir_trans_done_callback, ISR callback that is triggered when a transmission is done
        Index 1 source - ir_rec_handler, ISR triggered by the payload
    */

    // -- Init --
    // TX channel init
    rmt_channel_handle_t tx_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,   // select source clock
        .gpio_num = RMT_GPIO_NUM,         // GPIO number
        .mem_block_symbols = 64,          // memory block size, 64 * 4 = 256 Bytes
        .resolution_hz = IR_RESOLUTION_HZ, // 1 MHz tick resolution, i.e., 1 tick = 1 µs
        .trans_queue_depth = 4,           // set the number of transactions that can pend in the background
        .flags.with_dma = false,          // do not need DMA backend
    };
    ESP_ERROR_CHECK( rmt_new_tx_channel(&tx_chan_config, &tx_chan) );

    // Carrier config init
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan, &carrier_cfg));

    rmt_transmit_config_t rmt_transmit_config = {
        .loop_count = 0, // no loop
    };

    // RMT encoder init (mechanism that turns raw data into NEC RMT symbols)
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    // Register a callback for RMT disabling
    rmt_tx_event_callbacks_t rmt_tx_event_callback = {
        .on_trans_done = ir_trans_done_callback,
    };
    rmt_tx_register_event_callbacks(tx_chan, &rmt_tx_event_callback, NULL);

    // IR payload holder
    ir_nec_scan_code_t ir_nec_data = {0};

    // For debugging, comment out all tasks initializations in app_main except this one
    #if (IR_DEBUG_FLAG)
        ESP_LOGI(printerVar, "[WARNING] IR debug mode has been enabled");

        // Enable the RMT peripheral
        ESP_ERROR_CHECK( rmt_enable(tx_chan) );
        while( true ){
            ESP_LOGI(printerVar, "IR has been triggered");
            
            // Dummy payload
            ir_nec_data.address = 0x12;
            ir_nec_data.command = 0x26;
            
            // Send current signal
            ESP_ERROR_CHECK( rmt_transmit(tx_chan, nec_encoder, &ir_nec_data, sizeof(ir_nec_data), &rmt_transmit_config) );
            
            // [BLOCKING] Wait until the IR transmission is done
            ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

            vTaskDelay( 1000 / portTICK_PERIOD_MS );
        }
    #endif

    // -- Main logic --
    while( true ){
        // [BLOCKING] Wait for the Queue to get filled with IR payload data
        xQueuePeek(xQueueIRdata, &ir_nec_data, portMAX_DELAY);
        ESP_LOGI(printerVar, "IR has been triggered");
        
        // Enable the RMT peripheral
        ESP_ERROR_CHECK( rmt_enable(tx_chan) );
        
        // [BLOCKING] Send current command and Wait until the IR transmission is done
        ESP_ERROR_CHECK( rmt_transmit(tx_chan, nec_encoder, &ir_nec_data, sizeof(ir_nec_data), &rmt_transmit_config) );
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // In case command == Capture,
        // Keep resending the IR command until the payload is inside 
        if( ir_nec_data.command == 0x02 ){
            // Enable the pin interrupt (Disabled in the ISR)
            gpio_intr_enable(GATE_GPIO_NUM_REC);
            ESP_LOGI(printerVar, "PWM GATE Interrupt has been enabled");

            // [BLOCKING] Wait for the notification from the ISR triggered by the payload
            // Re-transmistt the command if the notification take command times out
            while( ulTaskNotifyTakeIndexed(1, pdTRUE, IR_CAPTURE_RESEND_WAIT/portTICK_PERIOD_MS) == 0 ){
                // [BLOCKING] Send and wait until the IR transmission is done
                ESP_ERROR_CHECK( rmt_transmit(tx_chan, nec_encoder, &ir_nec_data, sizeof(ir_nec_data), &rmt_transmit_config) );
                ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
                ESP_LOGI(printerVar, "The capture command is sent again");
            }
            ESP_LOGI(printerVar, "The notification from the payload was received");
        }

        // Clear the queue
        xQueueReceive(xQueueIRdata, &ir_nec_data, portMAX_DELAY);

        // Disable the RMT peripheral to save energy
        ESP_ERROR_CHECK( rmt_disable(tx_chan) );
    }
}