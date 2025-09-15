// cfg/config.h
#pragma once
#include "control/types.h"

/**
 * \file
 * \brief Application configuration model and persistence API.
 * \details
 * Exposes a small configuration container (\c AppConfig) that currently wraps
 * controller parameters (\c ControlParams), along with helpers to load/save
 * the configuration from EEPROM and to populate sane defaults.
 */

/**
 * \brief Top-level application configuration blob.
 * \details
 * Extend this struct as the firmware grows (additional feature blocks, calibration, etc.).
 */
struct AppConfig {
  /**
   * \brief Closed-loop controller parameters and thresholds.
   * \details
   * See \c control/types.h for field descriptions. Defaults convert human-readable
   * voltage thresholds to ADC counts.
   */
  ControlParams ctrl;
};

/**
 * \brief Load configuration from EEPROM or fall back to defaults.
 * \param[out] cfg Destination to populate.
 * \retval true  Loaded a valid record from EEPROM.
 * \retval false No valid record was found; \c cfg was filled with defaults and saved.
 */
bool config_load(AppConfig& cfg);

/**
 * \brief Populate \c cfg with safe defaults.
 * \param[out] cfg Configuration object to initialize.
 * \details
 * Calls \c controller_init_defaults() and overrides key thresholds/tunables.
 * Voltage thresholds are converted to ADC counts internally (volts→counts).
 */
void config_defaults(AppConfig& cfg);

/**
 * \brief Save configuration to EEPROM (wear-leveled).
 * \param cfg Configuration to persist.
 * \retval true  Save succeeded.
 * \retval false Save failed.
 */
bool config_save(const AppConfig& cfg);
