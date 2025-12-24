idf_component_register( SRCS "main.cpp" "wifi/wifi.cpp" "storage/storage.cpp" "sntp/sntp.cpp" "flyback/flyback.cpp" "commands/commands.cpp"
                        REQUIRES esp_timer freertos flyback_psu geiger_counter iot_is jobs esp_wifi esp_event nvs_flash esp_system driver ota
                        INCLUDE_DIRS "." "wifi" "storage" "sntp" "flyback" "commands")