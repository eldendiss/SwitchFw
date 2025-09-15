#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/**
 * \file
 * \brief RMT-based pulse counter for CPS (counts per second).
 * \details
 * This module implements a pulse counter using the ESP-IDF RMT RX peripheral (v5+).
 * It counts pulses on a specified GPIO pin by detecting valid high or low pulses.
 * Very short glitches are filtered out by the RX range filter; the \c min_pulse_ns
 * parameter defines the minimum pulse width accepted.
 */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * \brief Geiger counter device context.
   * \details
   * Holds configuration, internal buffers, synchronization primitives, and counters
   * for pulse counting and rate calculations.
   */
  typedef struct
  {
    rmt_channel_handle_t rx_chan; //!< RMT RX channel handle
    gpio_num_t gpio;              //!< GPIO number used for pulse input
    uint32_t resolution_hz;       //!< RMT tick rate (e.g., 20 MHz → 50 ns per tick)
    bool active_high;             //!< true to count HIGH pulses, false for LOW pulses
    uint32_t min_pulse_ns;        //!< Minimum pulse width in nanoseconds to accept

    // Internal buffers and synchronization
    rmt_symbol_word_t *buf; //!< Pointer to RMT symbol buffer
    size_t buf_symbols;     //!< Capacity of the buffer in symbols
    QueueHandle_t evt_q;    //!< Queue handle for RMT events
    TaskHandle_t task;      //!< Task handle for processing pulses
    SemaphoreHandle_t lock; //!< Mutex for thread-safe access

    // Counters
    uint64_t total; //!< Total pulses counted since start

    // CPS (counts per second) snapshot for delta calculation
    uint64_t last_total;  //!< Total count at last CPS calculation
    int64_t last_time_us; //!< Timestamp (microseconds) at last CPS calculation
    float last_cps;       //!< Last computed CPS value

    // CPM (counts per minute) rolling 60-second window
    uint32_t sec_counts[60]; //!< Circular buffer of counts per second
    uint32_t sec_sum;        //!< Sum of counts in the 60-second window
    uint32_t filled_secs;    //!< Number of valid seconds filled (≤ 60)
    uint32_t cur_sec_count;  //!< Accumulated counts in the current second
    size_t sec_head;         //!< Index of the most recent completed second
    int64_t last_sec_us;     //!< Timestamp (microseconds) of last second boundary

    volatile bool running; //!< Flag indicating if counting is active
  } geiger_counter_t;

  /**
   * \brief Create and start the Geiger counter.
   * \param dev            Pointer to zero-initialized geiger_counter_t struct.
   * \param gpio           Input GPIO number for tube pulses.
   * \param active_high    true to count HIGH pulses; false to count LOW pulses.
   * \param resolution_hz  RMT resolution frequency in Hz (recommend 20,000,000 for 2–3 µs pulses).
   * \param min_pulse_ns   Minimum pulse width in nanoseconds to accept (e.g., 800–1500 ns).
   * \param buf_symbols    Size of the RX buffer in symbols (e.g., 1024 or 2048).
   * \return ESP_OK on success, error code otherwise.
   */
  esp_err_t geiger_counter_start(geiger_counter_t *dev,
                                 gpio_num_t gpio,
                                 bool active_high,
                                 uint32_t resolution_hz,
                                 uint32_t min_pulse_ns,
                                 size_t buf_symbols);

  /**
   * \brief Stop the Geiger counter and free all allocated resources.
   * \param dev Pointer to geiger_counter_t struct.
   * \return ESP_OK on success, error code otherwise.
   */
  esp_err_t geiger_counter_stop(geiger_counter_t *dev);

  /**
   * \brief Get the total pulse count since the counter was started.
   * \param dev Pointer to geiger_counter_t struct.
   * \return Total pulse count (thread-safe).
   */
  uint64_t geiger_counter_get_total(geiger_counter_t *dev);

  /**
   * \brief Get counts per second (CPS) since the previous call.
   * \param dev Pointer to geiger_counter_t struct.
   * \return CPS value; first call returns 0 and initializes internal snapshot.
   * \details CPS is computed as the delta count divided by elapsed time since last call.
   */
  float geiger_counter_get_cps(geiger_counter_t *dev);

  /**
   * \brief Get counts per minute (CPM) over a rolling 60-second window.
   * \param dev Pointer to geiger_counter_t struct.
   * \return CPM value scaled by (60 / filled_secs) until the window fills.
   * \details CPM is computed by summing counts in the last 60 seconds.
   */
  float geiger_counter_get_cpm(geiger_counter_t *dev);

#ifdef __cplusplus
}
#endif
