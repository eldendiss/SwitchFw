#pragma once
#ifndef FLYBACK_H
#define FLYBACK_H

#include "flyback_psu.h"
#include "geiger_counter.h"

/**
 * @brief Initialize flyback PSU controller, configure GPIO pins and I2C interface.
 * 
 * @return esp_err_t 
 */
esp_err_t flyback_init();

/**
 * @brief Put flyback PSU into sleep mode. Note that I2C communication will not work in sleep mode.
 * 
 * @return esp_err_t 
 */
esp_err_t flyback_sleep();
/**
 * @brief Wake flyback PSU from sleep mode.
 * 
 * @return esp_err_t 
 */
esp_err_t flyback_wake();
/**
 * @brief Check if flyback PSU is awake.
 * 
 * @return true if awake, false if in sleep mode
 */
bool flyback_is_awake();

/**
 * @brief Enable flyback PSU output.
 * 
 * @return esp_err_t 
 */
esp_err_t flyback_enable();
/**
 * @brief Disable flyback PSU output.
 * 
 * @return esp_err_t 
 */
esp_err_t flyback_disable();
/**
 * @brief Check if flyback PSU output is enabled.
 * 
 * @return true if enabled, false if disabled
 */
bool flyback_is_enabled();

/**
 * @brief Set output voltage for specified channel.
 * 
 * @param channel Channel number (0..3)
 * @param voltage Voltage in volts
 * @return esp_err_t 
 */
esp_err_t flyback_set_voltage(uint8_t channel, float voltage);
/**
 * @brief Get output voltage for current channel.
 * 
 * @param voltage Voltage pointer to store the voltage in volts
 * @return esp_err_t 
 */
esp_err_t flyback_get_voltage(float *voltage);

/**
 * @brief Get current status of the flyback PSU.
 * 
 * @param status Pointer to a flyback_psu_status_t structure to store the status
 * @return esp_err_t 
 */
esp_err_t flyback_get_status(flyback_psu_status_t *status);

/**
 * @brief Set the active channel of the flyback PSU.
 * 
 * @param channel Channel number (0..3)
 * @return esp_err_t 
 */
esp_err_t flyback_set_channel(uint8_t channel);
/**
 * @brief Get the active channel of the flyback PSU.
 * 
 * @param channel 
 * @return esp_err_t 
 */
esp_err_t flyback_get_channel(uint8_t *channel);

/**
 * @brief Wait for the flyback PSU to become idle (not busy switching ranges).
 * 
 * @param timeout_ms Timeout in milliseconds
 * @param poll_interval_ms Poll interval in milliseconds
 * @return esp_err_t 
 */
esp_err_t flyback_wait_for_idle(uint32_t timeout_ms, uint32_t poll_interval_ms);

float flyback_counts_to_volts(uint16_t counts);
uint16_t flyback_volts_to_counts(float volts);

void flyback_set_geiger_counter_device(geiger_counter_pcnt4_t* dev);

#endif // FLYBACK_H