#include "geiger_counter.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "geiger_counter";

/**
 * \brief Event data structure for RX done callback.
 * \details Contains the number of RMT symbols received.
 */
typedef struct
{
    size_t num_symbols; //!< Number of symbols received in the event
} rx_event_t;

/**
 * \brief RMT RX done callback.
 * \param chan RMT channel handle.
 * \param edata Pointer to RX done event data.
 * \param user_ctx User context pointer (geiger_counter_t*).
 * \return true if a higher priority task was woken by sending to queue.
 * \details Called from ISR context when RMT RX completes a buffer.
 *          Sends an event to the geiger counter's event queue.
 */
static bool rx_done_cb(rmt_channel_handle_t chan,
                       const rmt_rx_done_event_data_t *edata,
                       void *user_ctx)
{
    geiger_counter_t *dev = (geiger_counter_t *)user_ctx;
    rx_event_t evt = {.num_symbols = edata->num_symbols};
    BaseType_t hp = pdFALSE;
    if (dev->evt_q)
        xQueueSendFromISR(dev->evt_q, &evt, &hp);
    return hp == pdTRUE;
}

/**
 * \brief Convert nanoseconds to RMT ticks based on resolution.
 * \param ns Nanoseconds to convert.
 * \param res_hz RMT resolution frequency in Hz.
 * \return Number of ticks (capped at 15-bit max 0x7FFF).
 */
static inline uint32_t ns_to_ticks(uint32_t ns, uint32_t res_hz)
{
    // ticks = ns * (res_hz / 1e9), rounded up
    uint64_t t = ((uint64_t)ns * (uint64_t)res_hz + 999999999ULL) / 1000000000ULL;
    return (t > 0x7FFFu) ? 0x7FFFu : (uint32_t)t; // 15-bit max per half-symbol
}

/**
 * \brief Count valid pulses in a buffer of RMT symbols.
 * \param dev Pointer to geiger_counter_t device context.
 * \param s Pointer to array of RMT symbols.
 * \param n Number of symbols in the array.
 * \return Number of valid pulses counted.
 * \details Counts pulses matching the configured active level and exceeding minimum pulse width.
 */
static size_t count_pulses(const geiger_counter_t *dev,
                           const rmt_symbol_word_t *s, size_t n)
{
    const uint32_t min_ticks = ns_to_ticks(dev->min_pulse_ns, dev->resolution_hz);
    const uint32_t act = dev->active_high ? 1u : 0u;
    size_t cnt = 0;

    for (size_t i = 0; i < n; ++i)
    {
        if (s[i].level0 == act && s[i].duration0 >= min_ticks)
            ++cnt;
        if (s[i].level1 == act && s[i].duration1 >= min_ticks)
            ++cnt;
    }
    return cnt;
}

/**
 * \brief Advance the rolling 60-second CPM window based on current time.
 * \param dev Pointer to geiger_counter_t device context.
 * \param now_us Current timestamp in microseconds.
 * \details Rolls the second buckets forward for each elapsed second,
 *          updating sums and resetting the current second count.
 */
static void roll_seconds(geiger_counter_t *dev, int64_t now_us)
{
    // Advance 1-second buckets even if no pulses arrive (called from RX task loop)
    while (now_us - dev->last_sec_us >= 1000000)
    {
        dev->sec_head = (dev->sec_head + 1) % 60;

        // Remove oldest second count from sum, add current second count, then clear current
        dev->sec_sum -= dev->sec_counts[dev->sec_head];
        dev->sec_counts[dev->sec_head] = dev->cur_sec_count;
        dev->sec_sum += dev->cur_sec_count;

        if (dev->filled_secs < 60)
            dev->filled_secs++;
        dev->cur_sec_count = 0;
        dev->last_sec_us += 1000000;
    }
}

/**
 * \brief Task function to process RMT RX events and count pulses.
 * \param arg Pointer to geiger_counter_t device context.
 * \details Waits for RX done events, counts pulses, updates counters,
 *          and restarts RMT reception.
 */
static void rx_task(void *arg)
{
    geiger_counter_t *dev = (geiger_counter_t *)arg;

    rmt_receive_config_t rxcfg = {
        .signal_range_min_ns = dev->min_pulse_ns,
        .signal_range_max_ns = 1000000, // 1 ms per half-symbol upper bound
    };

    // Start initial RMT receive
    (void)rmt_receive(dev->rx_chan, dev->buf,
                      dev->buf_symbols * sizeof(rmt_symbol_word_t),
                      &rxcfg);

    while (dev->running)
    {
        rx_event_t evt;
        if (xQueueReceive(dev->evt_q, &evt, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            size_t pulses = count_pulses(dev, dev->buf, evt.num_symbols);

            xSemaphoreTake(dev->lock, portMAX_DELAY);
            dev->total += pulses;
            dev->cur_sec_count += pulses;
            int64_t now = esp_timer_get_time();
            roll_seconds(dev, now);
            xSemaphoreGive(dev->lock);

            // Restart receive immediately for next batch
            (void)rmt_receive(dev->rx_chan, dev->buf,
                              dev->buf_symbols * sizeof(rmt_symbol_word_t),
                              &rxcfg);
        }
        else
        {
            // Timeout: roll zero-count seconds to keep CPM window fresh
            xSemaphoreTake(dev->lock, portMAX_DELAY);
            int64_t now = esp_timer_get_time();
            roll_seconds(dev, now);
            xSemaphoreGive(dev->lock);
        }
    }
    vTaskDelete(NULL);
}

/**
 * \brief Start the Geiger counter on a specified GPIO.
 * \param dev Pointer to zero-initialized geiger_counter_t struct.
 * \param gpio GPIO number for pulse input.
 * \param active_high true to count HIGH pulses, false for LOW pulses.
 * \param resolution_hz RMT resolution frequency in Hz (default 20 MHz if 0).
 * \param min_pulse_ns Minimum pulse width in nanoseconds to accept (default 1000 ns if 0).
 * \param buf_symbols Size of RMT RX buffer in symbols (default 1024 if 0).
 * \return ESP_OK on success, error code otherwise.
 */
esp_err_t geiger_counter_start(geiger_counter_t *dev,
                               gpio_num_t gpio,
                               bool active_high,
                               uint32_t resolution_hz,
                               uint32_t min_pulse_ns,
                               size_t buf_symbols)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev null");
    memset(dev, 0, sizeof(*dev));

    dev->gpio = gpio;
    dev->active_high = active_high;
    dev->resolution_hz = resolution_hz ? resolution_hz : 20000000u; // 20 MHz default
    dev->min_pulse_ns = min_pulse_ns ? min_pulse_ns : 1000u;        // 1 µs default
    dev->buf_symbols = buf_symbols ? buf_symbols : 1024u;

    // Configure RMT RX channel
    rmt_rx_channel_config_t cfg = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT, // APB clock (typically 80 MHz) derived
        .resolution_hz = dev->resolution_hz,
        .mem_block_symbols = dev->buf_symbols,
        .flags = {
            .invert_in = false,
            .with_dma = false}};
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&cfg, &dev->rx_chan), TAG, "new rx");
    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rx_done_cb};
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(dev->rx_chan, &cbs, dev), TAG, "cb");
    ESP_RETURN_ON_ERROR(rmt_enable(dev->rx_chan), TAG, "enable rx");

    // Allocate buffer for RMT symbols
    dev->buf = (rmt_symbol_word_t *)heap_caps_malloc(
        dev->buf_symbols * sizeof(rmt_symbol_word_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(dev->buf, ESP_ERR_NO_MEM, TAG, "no mem buf");

    // Create event queue and mutex lock
    dev->evt_q = xQueueCreate(4, sizeof(rx_event_t));
    ESP_RETURN_ON_FALSE(dev->evt_q, ESP_ERR_NO_MEM, TAG, "no mem queue");
    dev->lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(dev->lock, ESP_ERR_NO_MEM, TAG, "no mem mutex");

    // Initialize timing and counters
    dev->last_time_us = esp_timer_get_time();
    dev->last_sec_us = dev->last_time_us;
    dev->sec_head = 59; // Next roll will write index 0
    dev->filled_secs = 0;
    memset(dev->sec_counts, 0, sizeof(dev->sec_counts));

    dev->running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, "geiger_rx", 3072, dev, 9, &dev->task, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "no task");

    ESP_LOGI(TAG, "Started on GPIO %d, %s, res=%u Hz, min=%u ns, buf=%u symbols",
             (int)gpio, active_high ? "HIGH" : "LOW",
             (unsigned)dev->resolution_hz, (unsigned)dev->min_pulse_ns,
             (unsigned)dev->buf_symbols);
    return ESP_OK;
}

/**
 * \brief Stop the Geiger counter and free all resources.
 * \param dev Pointer to geiger_counter_t struct.
 * \return ESP_OK on success, error code otherwise.
 * \details Stops the RX task, disables and deletes the RMT channel,
 *          frees buffers, deletes queue and mutex.
 */
esp_err_t geiger_counter_stop(geiger_counter_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    dev->running = false;
    // Allow RX task to exit gracefully
    vTaskDelay(pdMS_TO_TICKS(20));

    if (dev->rx_chan)
    {
        rmt_disable(dev->rx_chan);
        rmt_del_channel(dev->rx_chan);
        dev->rx_chan = NULL;
    }
    if (dev->buf)
    {
        heap_caps_free(dev->buf);
        dev->buf = NULL;
    }
    if (dev->evt_q)
    {
        vQueueDelete(dev->evt_q);
        dev->evt_q = NULL;
    }
    if (dev->lock)
    {
        vSemaphoreDelete(dev->lock);
        dev->lock = NULL;
    }
    dev->task = NULL;
    return ESP_OK;
}

/**
 * \brief Get the total pulse count since the counter was started.
 * \param dev Pointer to geiger_counter_t struct.
 * \return Total pulse count (thread-safe).
 */
uint64_t geiger_counter_get_total(geiger_counter_t *dev)
{
    if (!dev)
        return 0;
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    uint64_t t = dev->total;
    xSemaphoreGive(dev->lock);
    return t;
}

/**
 * \brief Get counts per second (CPS) since the previous call.
 * \param dev Pointer to geiger_counter_t struct.
 * \return CPS value; first call returns 0 and initializes internal snapshot.
 * \details CPS is computed as the delta count divided by elapsed time since last call.
 */
float geiger_counter_get_cps(geiger_counter_t *dev)
{
    if (!dev)
        return 0.0f;
    int64_t now = esp_timer_get_time();

    xSemaphoreTake(dev->lock, portMAX_DELAY);
    uint64_t t = dev->total;
    uint64_t dtc = t - dev->last_total;
    int64_t dtu = now - dev->last_time_us;
    dev->last_total = t;
    dev->last_time_us = now;
    xSemaphoreGive(dev->lock);

    if (dtu <= 0)
        return dev->last_cps;
    float cps = (float)dtc * (1000000.0f / (float)dtu);
    dev->last_cps = cps;
    return cps;
}

/**
 * \brief Get counts per minute (CPM) over a rolling 60-second window.
 * \param dev Pointer to geiger_counter_t struct.
 * \return CPM value scaled by (60 / filled_secs) until the window fills.
 * \details CPM is computed by summing counts in the last 60 seconds.
 */
float geiger_counter_get_cpm(geiger_counter_t *dev)
{
    if (!dev)
        return 0.0f;
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    uint32_t sum = dev->sec_sum;
    uint32_t secs = dev->filled_secs ? dev->filled_secs : 1;
    xSemaphoreGive(dev->lock);

    // Scale to 60-second CPM if the window isn't full yet
    return (float)sum * (60.0f / (float)secs);
}
