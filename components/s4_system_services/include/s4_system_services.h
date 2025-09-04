#ifndef S4_SYSTEM_SERVICES_H
#define S4_SYSTEM_SERVICES_H

#include <stdint.h>

// Initializes system services like NVS (Non-Volatile Storage) and logging.
void s4_init_services(void);

// Manages low-power state transitions.
void s4_power_management_task(void);

// Logs an event to NVS or a ring buffer.
void s4_log_event(const char* event_message);

// --- Event logging (ring buffer) ---
typedef struct {
    uint16_t code;     // 100x=state, 200x=fault, 300x=HMI, 400x=calib, 500x=power
    int32_t  payload;  // Optional payload
    uint64_t ts_us;    // Timestamp (us)
} s4_event_t;

// Recommended size per spec
#ifndef S4_EVENT_RING_CAP
#define S4_EVENT_RING_CAP 512
#endif

// --- Event code definitions (subset) ---
#define EVT_STATE_CHANGE            1000
#define EVT_FAULT_DOUBLE_LIMIT      2001
#define EVT_FAULT_SENSOR_STALE      2002
#define EVT_FAULT_OVERCURRENT       2003
#define EVT_FAULT_ESTOP             2004
#define EVT_POWER_UVLO_ACTIVE       5001
#define EVT_POWER_UVLO_CLEAR        5002

void s4_log_event_code(uint16_t code, int32_t payload);
size_t s4_events_count(void);
size_t s4_events_read(s4_event_t* out, size_t max_items);


// --- NVS Parameter Management ---

/**
 * @brief Saves a 16-bit unsigned integer to NVS.
 * @param key The NVS key.
 * @param value The value to save.
 * @return ESP_OK on success, or an error code from NVS.
 */
esp_err_t s4_save_parameter_u16(const char* key, uint16_t value);

/**
 * @brief Loads a 16-bit unsigned integer from NVS.
 * @param key The NVS key.
 * @param default_value The value to return if the key is not found.
 * @return The loaded value or the default value.
 */
uint16_t s4_load_parameter_u16(const char* key, uint16_t default_value);

/**
 * @brief Saves a 32-bit float to NVS.
 * @param key The NVS key.
 * @param value The value to save.
 * @return ESP_OK on success, or an error code from NVS.
 */
esp_err_t s4_save_parameter_f32(const char* key, float value);

/**
 * @brief Loads a 32-bit float from NVS.
 * @param key The NVS key.
 * @param default_value The value to return if the key is not found.
 * @return The loaded value or the default value.
 */
float s4_load_parameter_f32(const char* key, float default_value);


#endif // S4_SYSTEM_SERVICES_H
