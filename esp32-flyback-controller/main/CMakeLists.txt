idf_component_register( SRCS    "main.cpp" "wifi/wifi.cpp" "storage/storage.cpp" 
                                "sntp/sntp.cpp" "flyback/flyback.cpp" "commands/commands.cpp" 
                                "BLE/ble.cpp" "BLE/gatt_svc.cpp" "BLE/gap.cpp" "provisioning/provisioning.cpp"
                                "hv_mon/hv_mon.cpp"
                        REQUIRES esp_timer freertos flyback_psu esp_adc geiger_counter LAN iot_is jobs esp_wifi esp_event nvs_flash esp_system driver ota bt esp_hw_support
                        INCLUDE_DIRS "." "wifi" "storage" "sntp" "flyback" "commands" "BLE" "provisioning" "hv_mon"
                        )