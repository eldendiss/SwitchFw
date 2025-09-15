idf_component_register( SRCS "main.c"
                        REQUIRES esp_timer freertos flyback_psu geiger_counter
                        INCLUDE_DIRS ".")