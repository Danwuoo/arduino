#ifndef S3_STATE_MACHINE_H
#define S3_STATE_MACHINE_H

#include "s1_sensing.h"
#include "s2_actuators.h"

// --- Section 4: Data Models & Contracts ---

// Safety-related inputs, gathered from limit switches and other critical checks.
typedef struct {
    bool limit_a_up;
    bool limit_a_down;
    bool limit_b_extended;
    bool limit_b_retracted;
    bool estop;
    bool overcurrent;
    bool undervoltage_lockout;
} safety_snap_t;

// Command for the actuator task. This is an output from the FSM.
typedef struct {
    motor_command_t motor_a;
    motor_command_t motor_b;
} actuator_cmd_t;

// System event for logging and potential future use.
typedef struct {
    uint16_t code;
    uint64_t timestamp;
    int32_t payload;
} event_t;


// The main states of the system, as per the blueprint.
typedef enum {
    STATE_IDLE,
    STATE_EXTENDING,
    STATE_RETRACTING,
    STATE_HOLD,
    STATE_DRY_DONE,
    STATE_FAULT
} system_state_t;

// Initializes the state machine.
void s3_init_fsm(void);

// The main "tick" function for the state machine.
// It takes sensor and safety data as input, and produces an actuator command as output.
void s3_fsm_tick(const sensor_data_t *sensors, const safety_snap_t *safety, actuator_cmd_t *cmd);

// Returns the current state of the system.
system_state_t s3_get_current_state(void);

#endif // S3_STATE_MACHINE_H
