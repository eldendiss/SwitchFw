#pragma once
#ifndef MAIN_H_
#define MAIN_H_

#include "iot_is.h"
#include "geiger_counter.h"
#include "flyback_psu.h"
#include "storage.h"

extern geiger_counter_pcnt4_t gc;
extern device_config_data_t devCfg;

#endif // MAIN_H_