#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>

#define HV_AVG_N 64

typedef struct {
    float buf[HV_AVG_N];
    uint32_t idx;
    uint32_t count;     // <= HV_AVG_N
    double sum;         // sum of valid samples
    float last;
    SemaphoreHandle_t mtx;
} hv_avg_t;

static hv_avg_t g_hv = {
    .idx = 0, .count = 0, .sum = 0.0, .last = NAN, .mtx = NULL
};

void hv_avg_init(void)
{
    if (!g_hv.mtx) g_hv.mtx = xSemaphoreCreateMutex();
}

void hv_avg_reset(void)
{
    if (!g_hv.mtx) hv_avg_init();
    xSemaphoreTake(g_hv.mtx, portMAX_DELAY);
    g_hv.idx = 0;
    g_hv.count = 0;
    g_hv.sum = 0.0;
    g_hv.last = NAN;
    // no need to clear buf, sum/count define validity
    xSemaphoreGive(g_hv.mtx);
}

void hv_avg_add(float v)
{
    if (!g_hv.mtx) hv_avg_init();
    xSemaphoreTake(g_hv.mtx, portMAX_DELAY);

    g_hv.last = v;

    if (g_hv.count < HV_AVG_N) {
        g_hv.buf[g_hv.idx] = v;
        g_hv.sum += v;
        g_hv.count++;
    } else {
        // overwrite oldest
        float old = g_hv.buf[g_hv.idx];
        g_hv.buf[g_hv.idx] = v;
        g_hv.sum += v - old;
    }

    g_hv.idx = (g_hv.idx + 1) % HV_AVG_N;

    xSemaphoreGive(g_hv.mtx);
}

float hv_avg_get(void)
{
    if (!g_hv.mtx) hv_avg_init();
    xSemaphoreTake(g_hv.mtx, portMAX_DELAY);
    float out = (g_hv.count == 0) ? NAN : (float)(g_hv.sum / (double)g_hv.count);
    xSemaphoreGive(g_hv.mtx);
    return out;
}

float hv_last_get(void)
{
    if (!g_hv.mtx) hv_avg_init();
    xSemaphoreTake(g_hv.mtx, portMAX_DELAY);
    float out = g_hv.last;
    xSemaphoreGive(g_hv.mtx);
    return out;
}