#include "s2_actuators.h"
#include "project_config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static const char *TAG = "S2_ACTUATORS";

// LEDC configuration for Motor A
#define LEDC_TIMER_A      LEDC_TIMER_0
#define LEDC_CHANNEL_A    LEDC_CHANNEL_0

// LEDC configuration for Motor B
#define LEDC_TIMER_B      LEDC_TIMER_1
#define LEDC_CHANNEL_B    LEDC_CHANNEL_1

/**
 * @brief Configures a GPIO pin as a standard input.
 */
static void configure_input_pin(gpio_num_t pin, bool pullup) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pullup,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/**
 * @brief Initializes motor control pins (PWM and Direction) and limit switches.
 */
void s2_init_actuators(void) {
    ESP_LOGI(TAG, "Initializing Actuator Layer...");

    // --- Configure Motor A Pins ---
    gpio_set_direction(MOTOR_A_DIR_PIN, GPIO_MODE_OUTPUT);

    // --- Configure Motor B Pins ---
    gpio_set_direction(MOTOR_B_DIR_PIN, GPIO_MODE_OUTPUT);

    // --- Configure Limit Switch Pins ---
    // Note: Pins 34-39 are input-only and do not have internal pull-up resistors.
    // External pull-ups are required for these specific pins.
    configure_input_pin(LIMIT_A_UP_PIN, false);  // No internal pull-up on GPIO34
    configure_input_pin(LIMIT_A_DN_PIN, false);  // No internal pull-up on GPIO35
    configure_input_pin(LIMIT_B_EXT_PIN, true); // Has internal pull-up
    configure_input_pin(LIMIT_B_RET_PIN, true); // Has internal pull-up
    configure_input_pin(ESTOP_IN_PIN, true);    // Has internal pull-up

    // --- Configure LEDC PWM for both motors ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_A,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .freq_hz          = MOTOR_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_timer.timer_num = LEDC_TIMER_B; // Use the same config for the second timer
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_a = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_A,
        .timer_sel      = LEDC_TIMER_A,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A_PWM_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_a);

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER_B,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B_PWM_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_b);

    ESP_LOGI(TAG, "Actuator Layer Initialized.");
}

/**
 * @brief Set the state of Motor A.
 */
void s2_set_motor_a(motor_command_t cmd) {
    // Note: Direction logic depends on H-Bridge wiring.
    // This assumes DIR=1 is FORWARD and DIR=0 is REVERSE.
    switch (cmd.mode) {
        case MOTOR_MODE_FORWARD:
            gpio_set_level(MOTOR_A_DIR_PIN, 1);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A, cmd.duty_cycle);
            break;
        case MOTOR_MODE_REVERSE:
            gpio_set_level(MOTOR_A_DIR_PIN, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A, cmd.duty_cycle);
            break;
        case MOTOR_MODE_STOP:
        case MOTOR_MODE_BRAKE: // Simple stop, no active braking implemented
        default:
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A, 0);
            break;
    }
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A);
    ESP_LOGD(TAG, "Motor A set to mode %d, duty %d", cmd.mode, cmd.duty_cycle);
}

/**
 * @brief Set the state of Motor B.
 */
void s2_set_motor_b(motor_command_t cmd) {
    switch (cmd.mode) {
        case MOTOR_MODE_FORWARD:
            gpio_set_level(MOTOR_B_DIR_PIN, 1);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B, cmd.duty_cycle);
            break;
        case MOTOR_MODE_REVERSE:
            gpio_set_level(MOTOR_B_DIR_PIN, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B, cmd.duty_cycle);
            break;
        case MOTOR_MODE_STOP:
        case MOTOR_MODE_BRAKE:
        default:
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B, 0);
            break;
    }
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B);
    ESP_LOGD(TAG, "Motor B set to mode %d, duty %d", cmd.mode, cmd.duty_cycle);
}

/**
 * @brief Immediately stops all motors.
 */
void s2_emergency_stop(void) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_A);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_B);
    ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED");
}

/**
 * @brief Reads the state of all limit switches. NC (Normally Closed) logic is assumed.
 * Pin reading 1 means switch is NOT triggered.
 * Pin reading 0 means switch IS triggered (circuit is open).
 */
void s2_get_safety_snap(safety_snap_t *snap) {
    if (!snap) return;

    // NC logic: triggered = 0, not triggered = 1. We invert the logic so true means triggered.
    snap->limit_a_up = (gpio_get_level(LIMIT_A_UP_PIN) == 0);
    snap->limit_a_down = (gpio_get_level(LIMIT_A_DN_PIN) == 0);
    snap->limit_b_extended = (gpio_get_level(LIMIT_B_EXT_PIN) == 0);
    snap->limit_b_retracted = (gpio_get_level(LIMIT_B_RET_PIN) == 0);
    snap->estop = (gpio_get_level(ESTOP_IN_PIN) == 0);

    // Placeholder for other safety checks
    snap->overcurrent = false;
    snap->undervoltage_lockout = false;
}
