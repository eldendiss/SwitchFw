#pragma once
#ifndef HV_MON_H
#define HV_MON_H


void hv_avg_init(void);

void hv_avg_reset(void);

void hv_avg_add(float v);

float hv_avg_get(void);

float hv_last_get(void);

#endif