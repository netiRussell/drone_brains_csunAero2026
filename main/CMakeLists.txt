idf_component_register(
                        SRCS "drone_brains_csunAero2026.c" "nec_ir/ir_nec_encoder.c" 
                        "tasks/ir_controller.c" "tasks/gate_controller.c" "tasks/uart_receiver.c"
                        "tasks/uart_sender.c" "common.c" 

                        INCLUDE_DIRS "."
                        REQUIRES esp_driver_uart mavlink esp_driver_ledc esp_driver_gpio esp_driver_rmt 
                        esp_driver_pcnt
                    )
