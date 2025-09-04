#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "driver/gpio.h"

// --- Section 3: I/O Pin Configuration ---
// I2C
#define I2C_SDA_PIN (GPIO_NUM_21)
#define I2C_SCL_PIN (GPIO_NUM_22)

// Motor A
#define MOTOR_A_PWM_PIN (GPIO_NUM_25)
#define MOTOR_A_DIR_PIN (GPIO_NUM_18)

// Motor B
#define MOTOR_B_PWM_PIN (GPIO_NUM_26)
#define MOTOR_B_DIR_PIN (GPIO_NUM_19)

// Inputs
#define RAIN_SENSOR_PIN (GPIO_NUM_4)
#define LIMIT_A_UP_PIN  (GPIO_NUM_34)
#define LIMIT_A_DN_PIN  (GPIO_NUM_35)
#define LIMIT_B_EXT_PIN (GPIO_NUM_32)
#define LIMIT_B_RET_PIN (GPIO_NUM_33)
#define ESTOP_IN_PIN    (GPIO_NUM_27)
#define WIND_PULSE_PIN  (GPIO_NUM_14) // Optional

// UI
#define UI_IO_PIN (GPIO_NUM_23)

// ADC Inputs
#define VBAT_ADC_PIN  (ADC1_CHANNEL_0) // GPIO 36 is ADC1_CH0
#define IA_ADC_PIN    (ADC1_CHANNEL_3) // GPIO 39 is ADC1_CH3
#define IB_ADC_PIN    (ADC1_CHANNEL_4) // GPIO 32 is ADC1_CH4, needs pin re-config if used with limit sw

// --- Section 6 & 8: Timing, Thresholds & Parameters ---
// PWM
#define MOTOR_PWM_FREQ_HZ (5000) // 5 kHz
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution (0-255)

// Timing
#define OVERCURRENT_DEBOUNCE_MS (200)
#define MOTOR_START_IMMUNITY_MS (50)
#define DUTY_SLOPE_MAX_PER_10MS (10) // 10% duty cycle change per 10ms
#define RAIN_DEBOUNCE_MS (1500)
#define SENSOR_LUX_MOVING_AVG_WINDOW (10)
#define UVLO_DEBOUNCE_MS (2000)

// Wind speed conversion: pulses per second to m/s (depends on sensor)
#ifndef WIND_PULSE_TO_MPS
#define WIND_PULSE_TO_MPS (1.0f) // Adjust per anemometer constant
#endif

// Thresholds (from Section 8, can be overridden by NVS)
#define DEFAULT_LUX_OPEN_THRESHOLD (15000)
#define DEFAULT_LUX_CLOSE_THRESHOLD (2000)
#define DEFAULT_RH_DONE_THRESHOLD (60.0f)
#define DEFAULT_WIND_CLOSE_THRESHOLD_MPS (9.0f)
#define DEFAULT_VBAT_LOCK_VOLTAGE (3.6f)
#define DEFAULT_VBAT_UNLOCK_VOLTAGE (3.7f)
#define DEFAULT_OVERCURRENT_SOFT_FACTOR (1.8f)
#define DEFAULT_OVERCURRENT_HARD_FACTOR (3.0f)

// --- Section 11: NVS Schema (Key Names) ---
#define NVS_KEY_CONFIG_VERSION "cfg_ver"
#define NVS_KEY_CALIB_V_SLOPE  "v_slope"
#define NVS_KEY_CALIB_V_OFFSET "v_offset"
#define NVS_KEY_LUX_OPEN       "lux_open"
#define NVS_KEY_LUX_CLOSE      "lux_close"
#define NVS_KEY_RH_DONE        "rh_done"
#define NVS_KEY_WIND_CLOSE     "wind_close"
#define NVS_KEY_WIND_OPEN      "wind_open"
#define NVS_KEY_VBAT_LOCK      "vbat_lock"
#define NVS_KEY_VBAT_UNLOCK    "vbat_unlock"
#define NVS_KEY_I_SOFT         "i_soft"
#define NVS_KEY_I_HARD         "i_hard"
#define NVS_KEY_DUTY_A_MAX     "duty_a_max"
#define NVS_KEY_DUTY_B_MAX     "duty_b_max"
#define NVS_KEY_ACCEL_SLOPE    "accel_slope"
#define NVS_KEY_SLEEP_DELAY_S  "sleep_delay"
#define NVS_KEY_LOG_LEVEL      "log_level"


#endif // PROJECT_CONFIG_H
