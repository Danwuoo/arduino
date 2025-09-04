#include "s3_state_machine.h"
#include "project_config.h"
#include "s4_system_services.h" // For NVS
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "S3_FSM";

// --- FSM State & Parameters ---
static system_state_t current_state = STATE_IDLE;

// Parameters loaded from NVS, with hardcoded defaults.
static uint16_t lux_open_threshold;
static uint16_t lux_close_threshold;
static float vbat_lock_voltage;
static float vbat_unlock_voltage;
static uint8_t duty_a_max;
static uint8_t duty_b_max;

// Timers/counters (in FSM tick periods, 100ms each)
static int uvlo_ticks = 0;               // counts time below lock voltage
static bool uvlo_active = false;
static bool uvlo_active_prev = false;
static int dry_ticks = 0;                // counts time RH below threshold
static int sensor_fail_ticks = 0;        // counts time since last good sensor
static uint64_t last_sensor_ts_us = 0;   // track last sensor timestamp seen


// Helper function to set motor commands
static void set_motor_command(actuator_cmd_t *cmd, motor_mode_t mode_a, uint8_t duty_a, motor_mode_t mode_b, uint8_t duty_b) {
    cmd->motor_a.mode = mode_a;
    cmd->motor_a.duty_cycle = duty_a;
    cmd->motor_b.mode = mode_b;
    cmd->motor_b.duty_cycle = duty_b;
}

void s3_init_fsm(void) {
    current_state = STATE_IDLE;

    // Load parameters from NVS, using defaults if not found.
    lux_open_threshold = s4_load_parameter_u16(NVS_KEY_LUX_OPEN, DEFAULT_LUX_OPEN_THRESHOLD);
    lux_close_threshold = s4_load_parameter_u16(NVS_KEY_LUX_CLOSE, DEFAULT_LUX_CLOSE_THRESHOLD);
    vbat_lock_voltage = s4_load_parameter_f32(NVS_KEY_VBAT_LOCK, DEFAULT_VBAT_LOCK_VOLTAGE);
    vbat_unlock_voltage = s4_load_parameter_f32(NVS_KEY_VBAT_UNLOCK, DEFAULT_VBAT_UNLOCK_VOLTAGE);
    duty_a_max = s4_load_parameter_u16(NVS_KEY_DUTY_A_MAX, 85);
    duty_b_max = s4_load_parameter_u16(NVS_KEY_DUTY_B_MAX, 85);

    ESP_LOGI(TAG, "State Machine initialized, current state: IDLE");
    ESP_LOGI(TAG, "Params: lux_open=%d, lux_close=%d, vbat_lock=%.2f, vbat_unlock=%.2f",
        lux_open_threshold, lux_close_threshold, vbat_lock_voltage, vbat_unlock_voltage);
}

void s3_fsm_tick(const sensor_data_t *sensors, const safety_snap_t *safety, actuator_cmd_t *cmd) {
    system_state_t next_state = current_state;

    // --- Track sensor freshness & failures ---
    if (sensors->lux >= 0.0f) {
        last_sensor_ts_us = sensors->timestamp;
        sensor_fail_ticks = 0;
    } else {
        sensor_fail_ticks++;
    }

    // --- UVLO debounce ---
    if (sensors->vbat < vbat_lock_voltage) {
        if (uvlo_ticks * 100 < UVLO_DEBOUNCE_MS) uvlo_ticks++;
    } else {
        uvlo_ticks = 0;
    }
    uvlo_active = (uvlo_ticks * 100 >= UVLO_DEBOUNCE_MS);
    if (uvlo_active && !uvlo_active_prev) {
        s4_log_event_code(EVT_POWER_UVLO_ACTIVE, 0);
    } else if (!uvlo_active && uvlo_active_prev) {
        s4_log_event_code(EVT_POWER_UVLO_CLEAR, 0);
    }
    uvlo_active_prev = uvlo_active;

    // --- Dryness accumulation (simple continuous 15 min below threshold) ---
    if (sensors->rh < DEFAULT_RH_DONE_THRESHOLD) {
        if (dry_ticks < (15 * 60 * 10)) dry_ticks++; // 100ms tick => 10 ticks/sec
    } else {
        dry_ticks = 0;
    }

    // --- High Priority Fault Checks (apply to all states) ---
    bool double_limit_conflict = (safety->limit_a_up && safety->limit_a_down) ||
                                 (safety->limit_b_extended && safety->limit_b_retracted);
    bool sensor_stale_60s = false;
    if (last_sensor_ts_us > 0) {
        uint64_t now = esp_timer_get_time();
        sensor_stale_60s = (now - last_sensor_ts_us) > (uint64_t)60 * 1000000ULL;
    } else if (sensor_fail_ticks * 100 >= 60000) {
        sensor_stale_60s = true;
    }

    bool fault_due_to_estop = safety->estop;
    bool fault_due_to_overcurrent = safety->overcurrent;
    bool fault_due_to_double_limit = double_limit_conflict;
    bool fault_due_to_sensor = sensor_stale_60s;

    if (fault_due_to_estop || fault_due_to_overcurrent || fault_due_to_double_limit || fault_due_to_sensor) {
        next_state = STATE_FAULT;
    }

    // --- State-specific logic ---
    switch (current_state) {
        case STATE_IDLE:
        case STATE_HOLD:
        case STATE_DRY_DONE:
            // Medium priority: Check for retract conditions
            if (sensors->rain || sensors->lux < lux_close_threshold || uvlo_active) {
                next_state = STATE_RETRACTING;
            }
            // Low priority: Check for extend conditions
            else if (sensors->lux > lux_open_threshold && !sensors->rain && sensors->vbat >= vbat_unlock_voltage) {
                next_state = STATE_EXTENDING;
            }
            break;

        case STATE_EXTENDING:
            // Check for retract conditions first
            if (sensors->rain || sensors->lux < lux_close_threshold || uvlo_active) {
                next_state = STATE_RETRACTING;
            }
            // Check if limits are reached
            else if (safety->limit_a_up && safety->limit_b_extended) {
                next_state = STATE_HOLD;
            }
            break;

        case STATE_RETRACTING:
            // Check if retract is complete
            if (safety->limit_a_down && safety->limit_b_retracted) {
                next_state = STATE_HOLD;
            }
            break;

        case STATE_FAULT:
            // Check for fault clear condition (e.g., E-stop released and voltage recovered)
            if (!safety->estop && sensors->vbat >= vbat_unlock_voltage) {
                next_state = STATE_IDLE;
            }
            break;
    }

    // --- State Transition & Action ---
    if (next_state != current_state) {
        ESP_LOGI(TAG, "State transition from %d to %d", current_state, next_state);
        s4_log_event_code(EVT_STATE_CHANGE, ((int)current_state << 16) | (int)next_state);
        if (next_state == STATE_FAULT) {
            if (fault_due_to_double_limit) s4_log_event_code(EVT_FAULT_DOUBLE_LIMIT, 0);
            if (fault_due_to_sensor)       s4_log_event_code(EVT_FAULT_SENSOR_STALE, 0);
            if (fault_due_to_overcurrent)  s4_log_event_code(EVT_FAULT_OVERCURRENT, 0);
            if (fault_due_to_estop)        s4_log_event_code(EVT_FAULT_ESTOP, 0);
        }
        current_state = next_state;
    }

    // --- Set motor commands based on the *new* state ---
    // Default to stopping motors
    set_motor_command(cmd, MOTOR_MODE_STOP, 0, MOTOR_MODE_STOP, 0);

    switch (current_state) {
        case STATE_EXTENDING:
            // If a limit is hit, stop that motor, otherwise run it.
            set_motor_command(cmd,
                safety->limit_a_up ? MOTOR_MODE_STOP : MOTOR_MODE_FORWARD, (uint8_t)((duty_a_max * 255) / 100),
                safety->limit_b_extended ? MOTOR_MODE_STOP : MOTOR_MODE_FORWARD, (uint8_t)((duty_b_max * 255) / 100));
            break;

        case STATE_RETRACTING:
            set_motor_command(cmd,
                safety->limit_a_down ? MOTOR_MODE_STOP : MOTOR_MODE_REVERSE, (uint8_t)((duty_a_max * 255) / 100),
                safety->limit_b_retracted ? MOTOR_MODE_STOP : MOTOR_MODE_REVERSE, (uint8_t)((duty_b_max * 255) / 100));
            break;

        case STATE_IDLE:
        case STATE_HOLD:
        case STATE_DRY_DONE:
        case STATE_FAULT:
            // All stop states
            set_motor_command(cmd, MOTOR_MODE_STOP, 0, MOTOR_MODE_STOP, 0);
            break;
    }

    // If dryness condition satisfied for 15 minutes, reflect state
    if (dry_ticks >= (15 * 60 * 10) && current_state != STATE_FAULT) {
        current_state = STATE_DRY_DONE;
    }
}

system_state_t s3_get_current_state(void) {
    return current_state;
}
