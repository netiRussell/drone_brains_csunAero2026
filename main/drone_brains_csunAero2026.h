#ifndef MAIN_H
#define MAIN_H

#include "common.h"
 
// --- ISRs ---
static void IRAM_ATTR ir_rec_handler(void* arg){
    // Debugging feature. Make sure all the other tasks are commented out
    #if( PWM_GATE_REC_DEBUG_FLAG )
        debug_only_counter++;
        //return;
    #endif

    // - Main logic -
    BaseType_t high_task_wakeup = pdFALSE;
    
    // Notify the IR handler that the payload has entered the pick-up mechanism
    vTaskNotifyGiveIndexedFromISR(ir_taskHandler, 1, &high_task_wakeup);

    // Stop more interrupts from this pin until the gate is lowered again
    gpio_intr_disable(GATE_GPIO_NUM_REC);
    
    // GPIO ISR infrastructure requires manual high priority task worken yielding.
    if (high_task_wakeup) {
        portYIELD_FROM_ISR();  // yield so IR task can run immediately
    }
}

#endif