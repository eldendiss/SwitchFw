#pragma once
#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "iot_is.h"

bool delayCommand(const std::vector<double> &params);
bool setIntervalCommand(const std::vector<double> &params);
bool setTube_command(const std::vector<double> &params);
bool setVoltage_r1_command(const std::vector<double> &params);
bool setVoltage_r2_command(const std::vector<double> &params);
bool setVoltage_r3_command(const std::vector<double> &params);
bool setVoltage_r4_command(const std::vector<double> &params);
bool setConversion_r1_command(const std::vector<double> &params);
bool setConversion_r2_command(const std::vector<double> &params);
bool setConversion_r3_command(const std::vector<double> &params);
bool setConversion_r4_command(const std::vector<double> &params);


#endif // COMMANDS_H_