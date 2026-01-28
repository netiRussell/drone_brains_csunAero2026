#ifndef GATECONTROLLER_H 
#define GATECONTROLLER_H


// --- Includes ---
#include "common.h"

// -- ISRs --
bool pillar_limitter_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

// --- Function declarations ---
void gate_controller( void* pvParameters );


#endif 