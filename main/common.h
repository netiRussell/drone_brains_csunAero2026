#ifndef COMMON_H
#define COMMON_H

// -- Includes --
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
#include "nec_ir/ir_nec_encoder.h"

#include <c_library_v2/common/mavlink.h>


// -- Macros --
#define FALSE   0
#define TRUE    1

#define UART_TX_PIN_NUM 43 // D6 on XIAO = TX
#define UART_RX_PIN_NUM 44 // D7 on XIAO = RX
#define UART_RTS_PIN_NUM 8 
#define UART_CTS_PIN_NUM 9
#define UART_PORT_NUM UART_NUM_2
#define UART_DELAY_TIME  500 // ms

#define MAVLINK_SYSTEM_ID 2
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_ONBOARD_COMPUTER 

// ALT_GATE_THRESHOLD is always < ALT_AIRBORNE_THRESHOLD
#define ALT_AIRBORNE_THRESHOLD -500
#define ALT_GATE_THRESHOLD -1200 

#define GATE_GPIO_NUM_LIM   3 // Sends the PWM signal, D2
#define PWM_GATE_GPIO_NUM_SIG   5 // Sends the PWM signal, D4
#define GATE_GPIO_NUM_REC   6 // Gets triggered when the payload is inside the pick-up mechanism, D5
#define PWM_GATE_TIMER_ID       LEDC_TIMER_0
#define PWM_GATE_TIMER_RES      LEDC_TIMER_12_BIT
#define PWM_GATE_SPEED_MODE     LEDC_LOW_SPEED_MODE
#define PWM_GATE_CHANNEL        LEDC_CHANNEL_0
#define PWM_GATE_ACTIVE_TIME    200 // ms
#define PWM_GATE_DELIVERY_WAIT  3000 // ms
#define PWM_GATE_STOP_WAIT      1000 // ms
#define PWM_GATE_QUEUE_WAIT     1000 // ms

#define RMT_GPIO_NUM        4 // Sends the IR signal, D3
#define IR_RESOLUTION_HZ    1000000 // 1Mhz, Period = 1micro sec.
#define IR_CAPTURE_RESEND_WAIT 1000 // ms

// Set only one flag at a time (besides the UART_DEBUG_FLAG and MAVLINK_DEBUG_FLAG)
#define PWM_GATE_REC_DEBUG_FLAG         0 // 1 = True, anything else = False 
#define PWM_GATE_SIG_DEBUG_FLAG         1 // 1 = True, anything else = False
#define IR_DEBUG_FLAG                   0 // 1 = True, anything else = False
#define MAVLINK_DEBUG_FLAG              0 // 1 = True, anything else = False
#define UART_DEBUG_FLAG                 1 // 1 = True, anything else = False
#define GATE_IR_DEBUG_FLAG              0 // 1 = True, anything else = False


// -- Printer pointer --  
extern const char* printerVar; // TO BE DELETED AFTER DEBUGGING: used to print out msgs to the terminal


// -- Queues --
extern QueueHandle_t xQueueIRdata;


// -- Task handlers --
extern TaskHandle_t uart_sender_taskHandler;
extern TaskHandle_t uart_receiver_taskHandler;
extern TaskHandle_t gate_taskHandler;
extern TaskHandle_t ir_taskHandler;


// -- States and state functions ( To be modified only with Mutexes/Spinlocks ) --
typedef enum {
    STATE_AIRBORNE,
    STATE_LOW_ALT,
    STATE_LANDED
} states_t;
extern states_t state;

extern portMUX_TYPE state_spinlock;

states_t check_state();

void update_state_to(states_t new_state);


// --- Helper functions ---
void handle_mavlink_msg( mavlink_message_t* msg );

#endif