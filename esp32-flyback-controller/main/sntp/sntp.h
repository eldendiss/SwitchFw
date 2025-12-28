#ifndef SNTP_H_
#define SNTP_H_

#include "esp_sntp.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_sntp(uint32_t syncIntervalMs);

#ifdef __cplusplus
}
#endif
#endif // SNTP_H_