#ifndef IRCONTROLLER_H
#define IRCONTROLLER_H


// -- Includes --
#include "common.h"

// -- ISRs --
bool ir_trans_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);

// -- Function declarations --
void ir_controller( void* pvParameters );


#endif 