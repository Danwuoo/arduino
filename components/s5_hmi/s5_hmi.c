#include "s5_hmi.h"
#include "esp_log.h"

static const char *TAG = "S5_HMI";

/**
 * @brief Initializes HMI components.
 */
void s5_init_hmi(void) {
    ESP_LOGI(TAG, "Initializing HMI Layer...");
    // I2C/GPIO initialization for LCD, buttons, etc. will go here.
}

/**
 * @brief Updates the LCD display with current system status.
 * @param state The current FSM state.
 * @param sensors A pointer to the latest sensor data.
 */
void s5_update_display(system_state_t state, const sensor_data_t *sensors) {
    // Logic to format and send data to the LCD.
    // Example: lcd_set_cursor(0, 0); lcd_printf("State: %d", state);
    ESP_LOGD(TAG, "Updating display. State: %d, Lux: %.1f", state, sensors->lux);
}

/**
 * @brief Checks for user input from buttons.
 */
void s5_check_buttons(void) {
    // Logic to read GPIOs for buttons.
    // This could trigger events for manual mode, fault clearing, etc.
}
