#include "s4_system_services.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "S4_SERVICES";
static const char *NVS_NAMESPACE = "system_cfg";

// --- Event ring buffer ---
static s4_event_t s_event_ring[S4_EVENT_RING_CAP];
static size_t s_event_head = 0; // next write
static size_t s_event_count = 0;
static portMUX_TYPE s_event_mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Initializes Non-Volatile Storage (NVS).
 */
void s4_init_services(void) {
    ESP_LOGI(TAG, "Initializing System Services (NVS)...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_LOGW(TAG, "NVS partition was corrupt or new version found, erasing...");
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Initialized.");
}

void s4_power_management_task(void) {
    ESP_LOGD(TAG, "Power management checks would run here.");
}

void s4_log_event(const char* event_message) {
    ESP_LOGI(TAG, "EVENT: %s", event_message);
}

void s4_log_event_code(uint16_t code, int32_t payload) {
    s4_event_t ev = {
        .code = code,
        .payload = payload,
        .ts_us = esp_timer_get_time(),
    };
    portENTER_CRITICAL(&s_event_mux);
    s_event_ring[s_event_head] = ev;
    s_event_head = (s_event_head + 1) % S4_EVENT_RING_CAP;
    if (s_event_count < S4_EVENT_RING_CAP) s_event_count++; // else overwrite oldest
    portEXIT_CRITICAL(&s_event_mux);
}

size_t s4_events_count(void) {
    size_t c;
    portENTER_CRITICAL(&s_event_mux);
    c = s_event_count;
    portEXIT_CRITICAL(&s_event_mux);
    return c;
}

size_t s4_events_read(s4_event_t* out, size_t max_items) {
    if (!out || max_items == 0) return 0;
    portENTER_CRITICAL(&s_event_mux);
    size_t cnt = s_event_count;
    size_t to_read = (cnt < max_items) ? cnt : max_items;
    // Oldest index
    size_t start = (s_event_head + S4_EVENT_RING_CAP - cnt) % S4_EVENT_RING_CAP;
    for (size_t i = 0; i < to_read; ++i) {
        out[i] = s_event_ring[(start + i) % S4_EVENT_RING_CAP];
    }
    portEXIT_CRITICAL(&s_event_mux);
    return to_read;
}


// --- NVS Parameter Management ---

esp_err_t s4_save_parameter_u16(const char* key, uint16_t value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u16(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

uint16_t s4_load_parameter_u16(const char* key, uint16_t default_value) {
    nvs_handle_t handle;
    uint16_t value = default_value;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return default_value;
    nvs_get_u16(handle, key, &value); // If key doesn't exist, 'value' remains unchanged.
    nvs_close(handle);
    return value;
}

esp_err_t s4_save_parameter_f32(const char* key, float value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    // NVS doesn't have a float type, so we save it as a 32-bit blob.
    err = nvs_set_blob(handle, key, &value, sizeof(float));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

float s4_load_parameter_f32(const char* key, float default_value) {
    nvs_handle_t handle;
    float value = default_value;
    size_t required_size = sizeof(float);
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) return default_value;
    nvs_get_blob(handle, key, &value, &required_size);
    nvs_close(handle);
    return value;
}
