#include "gate_controller.h"

void gate_controller( void* pvParameters ){
    /*
        [INFO] Notification Indexes
        Index 0 source - handle_mavlink_msg, whenever the drone has landed
        Index 1 source - pillar_limitter_handler, ISR triggered by the gate fully extending / retraction
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
    // Formula: number of max ticks in the clock * ratio of the clock that will stay High
    const uint32_t max_duty   = (1 << PWM_GATE_TIMER_RES) - 1; // Max value the clock reaches
    // Integer division is avoided by changing the formula into:
    const uint32_t duty_2ms   = (max_duty * 2000) / 20000; // full down
    const uint32_t duty_1ms   = (max_duty * 1000) / 20000; // full up
    const uint32_t duty_idle  = (max_duty * 1650) / 20000; // neutral

    // IR payload holder
    ir_nec_scan_code_t ir_nec_data = {0};

    // --- Main logic ---
    while( TRUE ){
        // Debugging feature. Make sure all the other tasks are commented out
        #if( PWM_GATE_SIG_DEBUG_FLAG )
            ESP_LOGI(printerVar, "[WARNING] PWM_GATE debug mode has been enabled");
            while ( TRUE ) {
                // Wake up the PWM peripheral &
                // Send the all Servo commands with 1sec delay in-between
                ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_idle);
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_idle) ); // Set the new PWM frequency
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_2ms );
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_2ms) ); // Set the new PWM frequency
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_idle);
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_idle) ); // Set the new PWM frequency
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_1ms );
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_1ms) ); // Set the new PWM frequency
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        #endif

        // Debugging feature. Make sure all the other tasks besides the IR_controller are commented out
        #if( GATE_IR_DEBUG_FLAG )
            ESP_LOGI(printerVar, "[WARNING] IR with PWM_GATE debug mode has been enabled");
            while( TRUE ){
                ESP_LOGI( printerVar, "\n\nNew cycle");

                // -- Pick-up the payload #2 (new) --
                // Send the "Capture" command to payload #2
                // [BLOCKING] the IR_Controller will block and resend the command until the payload is inside
                ir_nec_data.address = 0x02;
                ir_nec_data.command = 0x02;
                BaseType_t queue_status_debug = xQueueSend(xQueueIRdata, &ir_nec_data, portMAX_DELAY);
                if( queue_status_debug != pdTRUE){
                    ESP_LOGE(printerVar, "Queue Error: Capture command");
                }

                // -- Raise the gate --
                ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_1ms );

                // Wake up the PWM peripheral &
                // Send the corresponding Servo command
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_1ms) ); // Set the new PWM with 1000microsec high signal
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

                // Wait for the gate to fully retract itself
                vTaskDelay(PWM_GATE_ACTIVE_TIME/portTICK_PERIOD_MS); 

                // Reset and Put the peripheral back to sleep 
                ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_idle) ); // Reset the PWM 
                ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
                ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );
            }
        #endif

        // -- [BLOCKING] Wait for the notification --
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);

        // -- Lower the gate --
        ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_2ms );

        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_2ms) ); // Set the new PWM with 2000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

        // Wait for the gate to leave the top condcutive tape
        vTaskDelay(PWM_GATE_ACTIVE_TIME/portTICK_PERIOD_MS);

        // Enable the gate limitter pin interrupt (Disabled in the ISR)
        gpio_intr_enable(GATE_GPIO_NUM_LIM);

        // [BLOCKING] Wait until the gate is fully extended
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);

        // Reset and Put the peripheral back to sleep
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_idle) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );
       
        // -- Deliver the payload #1(current) --
        // [BLOCKING] Send the "Deliver" command to payload #1
        ir_nec_data.address = 0x01;
        ir_nec_data.command = 0x01;
        BaseType_t queue_status = xQueueSend(xQueueIRdata, &ir_nec_data, PWM_GATE_QUEUE_WAIT/portTICK_PERIOD_MS);
        if( queue_status != pdTRUE){
            ESP_LOGE(printerVar, "Queue Error: Delivery command");
        }

        // Wait a bit to ensure the payload #1 has exited the pick-up mechanism
        vTaskDelay(PWM_GATE_DELIVERY_WAIT/portTICK_PERIOD_MS);

        // -- Pick-up the payload #2 (new) --
        // Send the "Capture" command to payload #2
        // [BLOCKING] the IR_Controller will block and resend the command until the payload is inside
        ir_nec_data.address = 0x02;
        ir_nec_data.command = 0x02;
        queue_status = xQueueSend(xQueueIRdata, &ir_nec_data, PWM_GATE_QUEUE_WAIT/portTICK_PERIOD_MS);
        if( queue_status != pdTRUE){
            ESP_LOGE(printerVar, "Queue Error: Capture command");
        }

        // Send the "Stop" command to payload #2
        ir_nec_data.address = 0x02;
        ir_nec_data.command = 0x03;
        // [BLOCKING] Queue is full until the payload is inside, 
        // then the queue becomes empty and the STOP command can be sent
        queue_status = xQueueSend(xQueueIRdata, &ir_nec_data, portMAX_DELAY);
        if( queue_status != pdTRUE){
            ESP_LOGE(printerVar, "Queue Error: Stop command");
        }

        // Wait a bit to ensure the payload #2 has stopped inside the pick-up mechanism
        vTaskDelay(PWM_GATE_STOP_WAIT/portTICK_PERIOD_MS);

        // -- Raise the gate --
        ESP_LOGI( printerVar, "Gate controller has been triggered... duty = %ld", duty_1ms );

        // Wake up the PWM peripheral &
        // Send the corresponding Servo command
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_1ms) ); // Set the new PWM with 1000microsec high signal
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM

        // Wait for the gate to fully retract itself
        vTaskDelay(PWM_GATE_ACTIVE_TIME/portTICK_PERIOD_MS);

        // Enable the gate limitter interrupt pin (Disabled in the ISR)
        gpio_intr_enable(GATE_GPIO_NUM_LIM);

        // [BLOCKING] Wait until the gate is fully retracted 
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);

        // Reset and Put the peripheral back to sleep 
        ESP_ERROR_CHECK( ledc_set_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, duty_idle) ); // Reset the PWM 
        ESP_ERROR_CHECK( ledc_update_duty(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL) ); // Apply the new PWM
        ESP_ERROR_CHECK( ledc_stop(PWM_GATE_SPEED_MODE, PWM_GATE_CHANNEL, 0) );

        // Change the state to STATE_LOW_ALT
        update_state_to(STATE_LOW_ALT);
        ESP_LOGI(printerVar, "[!STATE CHANGE!]STATE_LANDED->STATE_LOW_ALT");
        
        // Allow UART_sender to send another request                        
        xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
    }
}

