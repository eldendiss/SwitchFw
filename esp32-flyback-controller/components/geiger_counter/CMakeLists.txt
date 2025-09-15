idf_component_register( SRCS "geiger_counter.c"
                        REQUIRES driver freertos esp_timer
                        INCLUDE_DIRS "include")