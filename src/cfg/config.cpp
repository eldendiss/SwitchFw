// cfg/config.cpp
#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "eeprom_store.h"
#include "../control/types.h"
#include "../control/controller.h"

// match your legacy scaling (431 * 2.5 = 1077.5)
static inline uint16_t counts_from_volts(float v){
  return (uint16_t)(v * (1023.0f / 1077.5f) + 0.5f);
}

static WearLevelStore<AppConfig, 128> g_store;  // 128B slots, runtime slot count

void config_defaults(AppConfig& cfg){
  controller_init_defaults(cfg.ctrl);

  cfg.ctrl.fb_set_cnt      = counts_from_volts(900.0f);
  cfg.ctrl.fb_ov_trip_cnt  = counts_from_volts(990.0f);
  cfg.ctrl.fb_ov_clear_cnt = counts_from_volts(950.0f);


  cfg.ctrl.err_deadband_cnt= counts_from_volts(1.5f);

  cfg.ctrl.pfm_enter_sub_cnt = counts_from_volts(12.0f);
  cfg.ctrl.pfm_lo_sub_cnt = counts_from_volts(10.0f);
}

bool config_load(AppConfig& cfg){
  if (g_store.load(cfg)) return true;
  config_defaults(cfg);
  g_store.save(cfg);
  return false;
}

bool config_save(const AppConfig& cfg){
  return g_store.save(cfg);
}
