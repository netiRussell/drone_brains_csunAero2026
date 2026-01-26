#include "common.h"


// -- Printer pointer --  
// [DEBUGGING] Define a char pointer for printing out
const char* printerVar = "printer"; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal

// -- Queues --
QueueHandle_t xQueueIRdata = NULL;


// -- Task handlers --
TaskHandle_t uart_sender_taskHandler = NULL;
TaskHandle_t uart_receiver_taskHandler = NULL;
TaskHandle_t gate_taskHandler = NULL;
TaskHandle_t ir_taskHandler = NULL;


// -- States and state functions ( To be modified only with Mutexes/Spinlocks ) --
states_t state = STATE_LOW_ALT;

portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;

states_t check_state(){
    // Create a holder for local, thread-safe copy
    states_t temp_s;

    taskENTER_CRITICAL(&state_spinlock);

    // [CRITICAL SECTION]
    temp_s = state;

    taskEXIT_CRITICAL(&state_spinlock);

    // Return the local copy
    return temp_s;
}

void update_state_to(states_t new_state){
    taskENTER_CRITICAL(&state_spinlock);

    // [CRITICAL SECTION]
    state = new_state;

    taskEXIT_CRITICAL(&state_spinlock);
}


// -- Helper functions --
void handle_mavlink_msg( mavlink_message_t* msg ){
    // Get current state
    states_t current_state = check_state();

    // Debugging feature
    #if ( MAVLINK_DEBUG_FLAG )         
        ESP_LOGI(printerVar, "Current state: %d", current_state);
        ESP_LOGI(printerVar, "msgid=%u seq=%u sys=%u comp=%u\n",
        msg->msgid, msg->seq, msg->sysid, msg->compid);
    #endif
    
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
            ESP_LOGI( printerVar, "[DATA] Relative altitude: %ld", rel_alt);
            
            switch(current_state){
                case STATE_AIRBORNE: {
                    // Above ALT_AIRBORNE_THRESHOLD
                    // Drone is airborne and has reached a significant altitude 

                    // Check if landed
                    if( rel_alt < ALT_GATE_THRESHOLD ){
                        ESP_LOGI(printerVar, "[!STATE CHANGE!] STATE_AIRBORNE->STATE_LANDED");

                        // Change the state to STATE_LANDED
                        update_state_to(STATE_LANDED);

                        // Notify the gate controller to lower the gate
                        xTaskNotifyGiveIndexed(gate_taskHandler, 0);

                        // Exit the function early
                        return;
                    } 
                }
                break;

                case STATE_LOW_ALT: {
                    // Below ALT_AIRBORNE_THRESHOLD
                    // Drone has recently or is about to take off

                    // Check if above certain altitude (ALT_AIRBORNE_THRESHOLD)
                    if( rel_alt > ALT_AIRBORNE_THRESHOLD ){
                        ESP_LOGI(printerVar, "[!STATE CHANGE!] STATE_LOW_ALT->STATE_AIRBORNE");

                        // Change the state to STATE_AIRBORNE
                        update_state_to(STATE_AIRBORNE);
                    }
                }
                break;

                case STATE_LANDED: {
                    // Below ALT_GATE_THRESHOLD(which is < ALT_AIRBORNE_THRESHOLD)
                    // Drone has landed and will pick-up the payload

                    /*
                        !Notice, the altitude request logic(uart_sender) is disabled in this state
                        
                        This state is updated to STATE_LOW_ALT in the gate_controller 
                        along with the semaphore update that allows UART_sender to 
                        continue sending altitude requests.

                        Enabling UART_sender signifies: drone is about to take off
                    */
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
            ESP_LOGI(printerVar, "Heart beat received.");
        }
        break;
        */
        
        default:
        {
            // ESP_LOGI(printerVar, "[WARNING] Incorrect msgid: %d", msg->msgid);
        }
        break;
    }

    // Allow UART_sender to send another request
    // [NOTE] to stay energy efficient, no UART communication during the payload pick-up procedure
    if( current_state != STATE_LANDED){
        xTaskNotifyGiveIndexed(uart_sender_taskHandler, 0);
    }
}
