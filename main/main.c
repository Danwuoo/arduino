#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "project_config.h"
#include "s0_power_protection.h"
#include "s1_sensing.h"
#include "s2_actuators.h"
#include "s3_state_machine.h"
#include "s4_system_services.h"
#include "s5_hmi.h"

static const char *TAG = "APP_MAIN";

// --- Task Handles ---
static TaskHandle_t xFsmTaskHandle = NULL;
static TaskHandle_t xSensorTaskHandle = NULL;
static TaskHandle_t xSafetyTaskHandle = NULL;
static TaskHandle_t xActuatorTaskHandle = NULL;
static TaskHandle_t xPowerTaskHandle = NULL;
static TaskHandle_t xLogTaskHandle = NULL;

// --- Queues for Inter-Task Communication (IPC) ---
static QueueHandle_t q_sensor_to_fsm; // Carries sensor_data_t
static QueueHandle_t q_safety_to_fsm; // Carries safety_snap_t
static QueueHandle_t q_fsm_to_actuator; // Carries actuator_cmd_t

// --- Task Prototypes ---
void safety_task(void *pvParameter);
void sensor_task(void *pvParameter);
void fsm_task(void *pvParameter);
void actuator_task(void *pvParameter);
void power_task(void *pvParameter);
void log_task(void *pvParameter);


void app_main(void) {
    ESP_LOGI(TAG, "===== System Initializing =====");

    // Create IPC primitives
    q_sensor_to_fsm = xQueueCreate(1, sizeof(sensor_data_t));
    q_safety_to_fsm = xQueueCreate(1, sizeof(safety_snap_t));
    q_fsm_to_actuator = xQueueCreate(1, sizeof(actuator_cmd_t));

    // Check if queues were created successfully
    if (q_sensor_to_fsm == NULL || q_safety_to_fsm == NULL || q_fsm_to_actuator == NULL) {
        ESP_LOGE(TAG, "Failed to create queues, halting.");
        while(1);
    }

    // Initialize system services and hardware components
    s4_init_services();
    s0_init();
    s1_init_sensors();
    s2_init_actuators();
    s3_init_fsm();
    s5_init_hmi(); // HMI task not created yet, but init is here

    ESP_LOGI(TAG, "===== Initialization Complete, Starting Tasks =====");

    // Create the FreeRTOS tasks
    xTaskCreate(safety_task,   "T_Safety",   2048, NULL, 4, &xSafetyTaskHandle);
    xTaskCreate(actuator_task, "T_Actuator", 2048, NULL, 4, &xActuatorTaskHandle);
    xTaskCreate(sensor_task,   "T_Sensors",  4096, NULL, 2, &xSensorTaskHandle);
    xTaskCreate(fsm_task,      "T_FSM",      4096, NULL, 3, &xFsmTaskHandle);
    xTaskCreate(power_task,    "T_Power",    2048, NULL, 2, &xPowerTaskHandle);
    xTaskCreate(log_task,      "T_Log",      3072, NULL, 1, &xLogTaskHandle);

    ESP_LOGI(TAG, "===== System is now Running =====");
}

/**
 * @brief T_Safety: High-priority task to check safety-critical inputs.
 * Period: 10ms (as per spec)
 */
void safety_task(void *pvParameter) {
    safety_snap_t snap;
    while (1) {
        s2_get_safety_snap(&snap);
        // Using xQueueOverwrite to always provide the FSM with the very latest data.
        xQueueOverwrite(q_safety_to_fsm, &snap);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief T_Sensors: Reads non-critical sensors and sends data to the FSM.
 * Period: 200ms (as per spec)
 */
void sensor_task(void *pvParameter) {
    sensor_data_t data;
    while (1) {
        s1_read_sensors(&data);
        xQueueOverwrite(q_sensor_to_fsm, &data);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief T_FSM: The core decision-making task.
 * It waits for data from other tasks and executes the state machine logic.
 */
void fsm_task(void *pvParameter) {
    sensor_data_t sensor_data;
    safety_snap_t safety_data;
    actuator_cmd_t actuator_cmd;

    // Wait for the first message from both queues to ensure we have valid data.
    if (!xQueueReceive(q_sensor_to_fsm, &sensor_data, pdMS_TO_TICKS(5000)) ||
        !xQueueReceive(q_safety_to_fsm, &safety_data, pdMS_TO_TICKS(5000))) {
        ESP_LOGE(TAG, "FSM task failed to receive initial data. Halting.");
        while(1);
    }
    ESP_LOGI(TAG, "FSM task received initial data, starting loop.");

    while (1) {
        // Loop at a fixed rate as specified in the design doc (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));

        // Peek at the queues to get the latest data without blocking.
        // This ensures the FSM is responsive and runs at a predictable rate.
        xQueuePeek(q_sensor_to_fsm, &sensor_data, 0);
        xQueuePeek(q_safety_to_fsm, &safety_data, 0);

        // Execute the state machine tick function
        s3_fsm_tick(&sensor_data, &safety_data, &actuator_cmd);

        // Send the resulting command to the actuator task
        xQueueOverwrite(q_fsm_to_actuator, &actuator_cmd);
    }
}

/**
 * @brief T_Actuator: Applies commands from the FSM to the motors.
 */
void actuator_task(void *pvParameter) {
    actuator_cmd_t target_cmd = {0};
    motor_command_t current_a = { .mode = MOTOR_MODE_STOP, .duty_cycle = 0 };
    motor_command_t current_b = { .mode = MOTOR_MODE_STOP, .duty_cycle = 0 };

    const uint8_t max_step = DUTY_SLOPE_MAX_PER_10MS; // per 10ms step
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Non-blocking fetch of latest target command
        actuator_cmd_t new_cmd;
        while (xQueueReceive(q_fsm_to_actuator, &new_cmd, 0) == pdTRUE) {
            target_cmd = new_cmd; // keep latest
        }

        // Enforce limit switch interlocks (never drive beyond hardware limits)
        safety_snap_t snap;
        s2_get_safety_snap(&snap);

        motor_command_t enforced_a = target_cmd.motor_a;
        motor_command_t enforced_b = target_cmd.motor_b;
        if ((enforced_a.mode == MOTOR_MODE_FORWARD && snap.limit_a_up) ||
            (enforced_a.mode == MOTOR_MODE_REVERSE && snap.limit_a_down)) {
            enforced_a.mode = MOTOR_MODE_STOP;
            enforced_a.duty_cycle = 0;
        }
        if ((enforced_b.mode == MOTOR_MODE_FORWARD && snap.limit_b_extended) ||
            (enforced_b.mode == MOTOR_MODE_REVERSE && snap.limit_b_retracted)) {
            enforced_b.mode = MOTOR_MODE_STOP;
            enforced_b.duty_cycle = 0;
        }

        // E-Stop immediate action
        if (snap.estop) {
            s2_emergency_stop();
            current_a.mode = MOTOR_MODE_STOP; current_a.duty_cycle = 0;
            current_b.mode = MOTOR_MODE_STOP; current_b.duty_cycle = 0;
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
            continue;
        }

        // Apply slope limiting on duty cycle towards enforced targets
        // Modes: if target is STOP, ramp down; if changing direction, ramp to 0 first then change mode
        // Motor A
        if (current_a.mode != enforced_a.mode) {
            // If not stopped, ramp down first
            if (current_a.duty_cycle > 0) {
                current_a.duty_cycle = (current_a.duty_cycle > max_step) ? (current_a.duty_cycle - max_step) : 0;
            } else {
                current_a.mode = (enforced_a.mode == MOTOR_MODE_BRAKE) ? MOTOR_MODE_STOP : enforced_a.mode;
            }
        } else {
            uint8_t tgt = enforced_a.duty_cycle;
            if (current_a.duty_cycle < tgt) {
                uint8_t inc = tgt - current_a.duty_cycle;
                current_a.duty_cycle += (inc > max_step) ? max_step : inc;
            } else if (current_a.duty_cycle > tgt) {
                uint8_t dec = current_a.duty_cycle - tgt;
                current_a.duty_cycle -= (dec > max_step) ? max_step : dec;
            }
        }

        // Motor B
        if (current_b.mode != enforced_b.mode) {
            if (current_b.duty_cycle > 0) {
                current_b.duty_cycle = (current_b.duty_cycle > max_step) ? (current_b.duty_cycle - max_step) : 0;
            } else {
                current_b.mode = (enforced_b.mode == MOTOR_MODE_BRAKE) ? MOTOR_MODE_STOP : enforced_b.mode;
            }
        } else {
            uint8_t tgt = enforced_b.duty_cycle;
            if (current_b.duty_cycle < tgt) {
                uint8_t inc = tgt - current_b.duty_cycle;
                current_b.duty_cycle += (inc > max_step) ? max_step : inc;
            } else if (current_b.duty_cycle > tgt) {
                uint8_t dec = current_b.duty_cycle - tgt;
                current_b.duty_cycle -= (dec > max_step) ? max_step : dec;
            }
        }

        // Apply to hardware
        s2_set_motor_a(current_a);
        s2_set_motor_b(current_b);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief T_Power: Low-power strategies, UVLO reactions, etc.
 * Period: 1000ms
 */
void power_task(void *pvParameter) {
    while (1) {
        s4_power_management_task();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief T_Log: Periodically drains the event ring buffer for diagnostics.
 * Period: 2000ms
 */
void log_task(void *pvParameter) {
    while (1) {
        size_t cnt = s4_events_count();
        if (cnt) {
            s4_event_t buf[32];
            size_t n = s4_events_read(buf, 32);
            for (size_t i = 0; i < n; ++i) {
                ESP_LOGI(TAG, "EVT code=%u payload=%ld ts=%llu", buf[i].code, (long)buf[i].payload, (unsigned long long)buf[i].ts_us);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
