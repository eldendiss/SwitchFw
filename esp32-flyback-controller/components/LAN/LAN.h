#pragma once
#ifndef LANC_H
#define LANC_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "mdns.h"
#include "esp_mac.h"

#define MOSI GPIO_NUM_13
#define MISO GPIO_NUM_12
#define SCK GPIO_NUM_14
#define CS GPIO_NUM_23
#define INT GPIO_NUM_4
#define RST GPIO_NUM_5

//callback called after gotip event
typedef void (*got_ip_callback_t)(void);

esp_err_t LAN_init();

void register_got_ip_callback(got_ip_callback_t callback);

void register_down_callback(got_ip_callback_t callback);

#endif