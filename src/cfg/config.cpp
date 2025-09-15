// cfg/config.cpp
#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "eeprom_store.h"
#include "../control/types.h"
#include "../control/controller.h"

/**
 * \file
 * \brief Configuration load/save and sane defaults for the PSU controller.
 * \details
 * - Provides a conversion helper from output volts to 10-bit ADC counts.
 * - Populates \c AppConfig with safe defaults.
 * - Loads/saves \c AppConfig via a wear-leveled EEPROM store.
 *
 * Units:
 * - Feedback thresholds are in **ADC counts** (0..1023).
 * - Times are in **milliseconds** unless stated otherwise.
 * - PWM/PFM duty is a **fraction** (0..1).
 */

/**
 * \brief Convert output voltage to 10-bit ADC counts (rounded).
 * \param v Output voltage in volts (expected 0..1077.5 V).
 * \return 10-bit ADC count (0..1023).
 * \details
 * Uses legacy scaling where 1077.5 V ≙ 1023 counts
 * (derived from historical \c 431 * 2.5 constant).
 * \note If you change the divider ratio or ADC reference, update 1077.5 accordingly.
 */
static inline uint16_t counts_from_volts(float v)
{
  return (uint16_t)(v * (1023.0f / 1077.5f) + 0.5f);
}

/**
 * \brief Wear-leveled EEPROM store for \c AppConfig.
 * \details
 * Slot size is 128 bytes; the number of usable slots is determined at runtime
 * by \c WearLevelStore (see \c eeprom_store.h for details).
 */
static WearLevelStore<AppConfig, 128> g_store; // 128B slots, runtime slot count

/**
 * \brief Populate configuration with safe defaults.
 * \param[out] cfg Configuration object to initialize.
 * \details
 * Calls \c controller_init_defaults() to fill controller params, then overrides:
 * - \c fb_set_cnt:      nominal output setpoint (≈900 V)
 * - \c fb_ov_trip_cnt:  over-voltage trip (≈990 V)
 * - \c fb_ov_clear_cnt: OV clear threshold (≈950 V)
 * - \c err_deadband_cnt: error deadband (~1.5 V at output)
 * - \c pfm_lo_sub_cnt:       low-subthreshold for PFM (~10 V)
 * - \c pfm_enter_sub_cnt:    enter PFM threshold (~15 V)
 * - \c pfm_burst_ms:         burst on-time window (ms)
 * - \c pfm_burst_duty:       burst duty (0..1)
 * - \c pfm_min_off_ms:       minimum off-time between bursts (ms)
 */
void config_defaults(AppConfig &cfg)
{
  controller_init_defaults(cfg.ctrl);

  cfg.ctrl.fb_set_cnt_tab[0] = counts_from_volts(200.0f);
  cfg.ctrl.fb_ov_clear_cnt_tab[0] = counts_from_volts(200.0f + 30);
  cfg.ctrl.fb_ov_trip_cnt_tab[0] = counts_from_volts(200.0f + 50);

  cfg.ctrl.fb_set_cnt_tab[1] = counts_from_volts(300.0f);
  cfg.ctrl.fb_ov_clear_cnt_tab[1] = counts_from_volts(300.0f + 30);
  cfg.ctrl.fb_ov_trip_cnt_tab[1] = counts_from_volts(300.0f + 50);

  cfg.ctrl.fb_set_cnt_tab[2] = counts_from_volts(400.0f);
  cfg.ctrl.fb_ov_clear_cnt_tab[2] = counts_from_volts(400.0f + 30);
  cfg.ctrl.fb_ov_trip_cnt_tab[2] = counts_from_volts(400.0f + 50);

  cfg.ctrl.fb_set_cnt_tab[3] = counts_from_volts(500.0f);
  cfg.ctrl.fb_ov_clear_cnt_tab[3] = counts_from_volts(500.0f + 30);
  cfg.ctrl.fb_ov_trip_cnt_tab[3] = counts_from_volts(500.0f + 50);

  cfg.ctrl.fb_set_cnt = cfg.ctrl.fb_set_cnt_tab[0];
  cfg.ctrl.fb_ov_trip_cnt = cfg.ctrl.fb_ov_trip_cnt_tab[0];
  cfg.ctrl.fb_ov_clear_cnt = cfg.ctrl.fb_ov_clear_cnt_tab[0];

  cfg.ctrl.err_deadband_cnt = counts_from_volts(1.5f);

  cfg.ctrl.pfm_lo_sub_cnt = counts_from_volts(10.0f);
  cfg.ctrl.pfm_enter_sub_cnt = counts_from_volts(15.0f);
  cfg.ctrl.pfm_burst_ms = 2;
  cfg.ctrl.pfm_burst_duty = 0.02f;
  cfg.ctrl.pfm_min_off_ms = 300;
}

/**
 * \brief Load configuration from EEPROM, falling back to defaults if absent/invalid.
 * \param[out] cfg Destination configuration object.
 * \retval true  A valid record was found and loaded.
 * \retval false No valid record; defaults were applied and saved.
 */
bool config_load(AppConfig &cfg)
{
  if (g_store.load(cfg))
    return true;
  config_defaults(cfg);
  g_store.save(cfg);
  return false;
}

/**
 * \brief Save configuration to EEPROM using wear leveling.
 * \param cfg Configuration to persist.
 * \retval true  Save succeeded.
 * \retval false Save failed.
 */
bool config_save(const AppConfig &cfg)
{
  return g_store.save(cfg);
}
