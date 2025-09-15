// cfg/config.h
#pragma once
#include "control/types.h"

struct AppConfig {
  ControlParams ctrl;
};

bool config_load(AppConfig& cfg);              // from EEPROM or defaults
void config_defaults(AppConfig& cfg);          // volts→counts
bool config_save(const AppConfig& cfg);
