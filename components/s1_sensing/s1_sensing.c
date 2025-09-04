#include "s1_sensing.h"
#include "project_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "s4_system_services.h"

static const char *TAG = "S1_SENSING";

// BH1750 I2C definitions
#define BH1750_SENSOR_ADDR (0x23)
#define BH1750_CMD_POWER_ON (0x01)
#define BH1750_CMD_H_RES_MODE (0x10)

// --- Local function prototypes ---
static esp_err_t i2c_master_init(void);
static esp_err_t bh1750_read_lux(float *lux);
static void adc1_init(void);
static void rain_sensor_init(void);
static bool rain_debounce_update(bool raw_now, int64_t now_us);
static float lux_filter_add(float lux_now);
static void wind_sensor_init(void);
static void IRAM_ATTR wind_isr_handler(void* arg);

/**
 * @brief Initializes the sensor components.
 */
void s1_init_sensors(void) {
    ESP_LOGI(TAG, "Initializing Sensing Layer...");
    ESP_ERROR_CHECK(i2c_master_init());
    adc1_init();
    rain_sensor_init();
    wind_sensor_init();
    // Note: DHT22 initialization would go here, likely requiring a third-party library.
    ESP_LOGI(TAG, "Sensing Layer Initialized.");
}

/**
 * @brief Reads data from all sensors.
 */
void s1_read_sensors(sensor_data_t *data) {
    if (!data) return;

    float lux_now = 0.0f;
    if (bh1750_read_lux(&lux_now) == ESP_OK && lux_now >= 0.0f) {
        data->lux = lux_filter_add(lux_now);
    } else {
        // Keep previous filtered value if read fails; mark with negative raw by convention
        ESP_LOGW(TAG, "BH1750 read error; retaining previous filtered lux");
        // data->lux remains last filtered value (lux_filter_add does not run)
    }

    int adc_reading = adc1_get_raw(VBAT_ADC_PIN);
    // Convert ADC code to voltage (rough; ESP32 ADC non-linear without calibration)
    // Assuming ADC width 12-bit, attenuation 11dB (~3.3V FS at pin)
    float v_adc = (float)adc_reading * (3.3f / 4095.0f);
    // Project uses a divider to VBAT; default assumes 1:1 (10k/10k) => factor 2.0
    float v_bat_meas = v_adc * 2.0f;
    // Apply user calibration from NVS
    data->vbat = v_bat_meas * s4_load_parameter_f32(NVS_KEY_CALIB_V_SLOPE, 1.0f)
               + s4_load_parameter_f32(NVS_KEY_CALIB_V_OFFSET, 0.0f);

    // Rain sensor: board outputs 1=dry, 0=wet. Debounced 1–2s window.
    int64_t now_us = esp_timer_get_time();
    bool raw_rain_wet = (gpio_get_level(RAIN_SENSOR_PIN) == 0);
    data->rain = rain_debounce_update(raw_rain_wet, now_us);

    // Wind speed from pulse count
    uint32_t pulses = s_wind_pulse_count;
    int64_t dt_us = now_us - s_wind_last_calc_us;
    if (dt_us > 200000) { // update every >=200ms window
        uint32_t dp = pulses - s_wind_last_pulse_count;
        float pps = (float)dp / ((float)dt_us / 1000000.0f);
        data->wind_speed = pps * WIND_PULSE_TO_MPS; // conversion constant depends on sensor
        s_wind_last_calc_us = now_us;
        s_wind_last_pulse_count = pulses;
    }

    // --- Placeholders for unimplemented sensors ---
    data->rh = 55.0f; // Placeholder
    data->temp = 25.0f; // Placeholder
    data->current_motor_a = 0.0f; // Placeholder
    data->current_motor_b = 0.0f; // Placeholder

    data->timestamp = esp_timer_get_time();

    ESP_LOGD(TAG, "Sensors Read: Lux=%.1f, VBat=%.2fV, Rain=%d", data->lux, data->vbat, data->rain);
}


// --- Helper function implementations ---

/**
 * @brief Initialize the I2C master bus.
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz
    };
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/**
 * @brief Read the light intensity from the BH1750 sensor.
 */
static esp_err_t bh1750_read_lux(float *lux) {
    uint8_t data_h, data_l;

    // Send power on and measurement mode command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_CMD_H_RES_MODE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 command failed");
        *lux = -1.0; // Indicate error
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(180)); // Wait for measurement

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data_h, I2C_ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, I2C_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *lux = ((data_h << 8) | data_l) / 1.2;
    } else {
        ESP_LOGE(TAG, "BH1750 read failed");
        *lux = -1.0; // Indicate error
    }
    return ret;
}

/**
 * @brief Initialize ADC1 for battery voltage monitoring.
 */
static void adc1_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VBAT_ADC_PIN, ADC_ATTEN_DB_11);
}

// --- Local filter/debounce state ---
static float s_lux_sum = 0.0f;
static float s_lux_window[ SENSOR_LUX_MOVING_AVG_WINDOW > 0 ? SENSOR_LUX_MOVING_AVG_WINDOW : 1 ];
static uint8_t s_lux_idx = 0;
static uint8_t s_lux_cnt = 0;

static bool s_rain_state = false;           // Debounced state: true = wet
static bool s_rain_last_raw = false;
static int64_t s_rain_last_change_us = 0;

static volatile uint32_t s_wind_pulse_count = 0;
static int64_t s_wind_last_calc_us = 0;
static uint32_t s_wind_last_pulse_count = 0;

static float lux_filter_add(float lux_now) {
    // Initialize buffer gradually
    if (s_lux_cnt < SENSOR_LUX_MOVING_AVG_WINDOW) {
        s_lux_sum += lux_now;
        s_lux_window[s_lux_idx++] = lux_now;
        s_lux_cnt++;
        if (s_lux_idx >= SENSOR_LUX_MOVING_AVG_WINDOW) s_lux_idx = 0;
        return s_lux_sum / s_lux_cnt;
    } else {
        s_lux_sum -= s_lux_window[s_lux_idx];
        s_lux_window[s_lux_idx++] = lux_now;
        if (s_lux_idx >= SENSOR_LUX_MOVING_AVG_WINDOW) s_lux_idx = 0;
        s_lux_sum += lux_now;
        return s_lux_sum / SENSOR_LUX_MOVING_AVG_WINDOW;
    }
}

static bool rain_debounce_update(bool raw_now, int64_t now_us) {
    if (raw_now != s_rain_last_raw) {
        s_rain_last_raw = raw_now;
        s_rain_last_change_us = now_us;
    }
    int64_t elapsed = now_us - s_rain_last_change_us;
    if (elapsed >= (int64_t)RAIN_DEBOUNCE_MS * 1000) {
        s_rain_state = s_rain_last_raw; // accept new state after debounce window
    }
    return s_rain_state;
}

static void wind_sensor_init(void) {
    // Configure pulse input and ISR on rising edge
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WIND_PULSE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(WIND_PULSE_PIN, wind_isr_handler, NULL);
    s_wind_last_calc_us = esp_timer_get_time();
}

static void IRAM_ATTR wind_isr_handler(void* arg) {
    (void)arg;
    s_wind_pulse_count++;
}

/**
 * @brief Initialize the GPIO for the rain sensor.
 */
static void rain_sensor_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RAIN_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true, // Enable pull-up for defined state when dry
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}
