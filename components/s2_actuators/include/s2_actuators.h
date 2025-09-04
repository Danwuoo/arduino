#ifndef S2_ACTUATORS_H
#define S2_ACTUATORS_H

#include <stdint.h>
#include <stdbool.h>
#include "s3_state_machine.h" // For safety_snap_t

// Defines the state of a single motor.
typedef enum {
    MOTOR_MODE_STOP,    // Motor is stopped (PWM is 0)
    MOTOR_MODE_FORWARD, // Motor is moving in one direction
    MOTOR_MODE_REVERSE, // Motor is moving in the opposite direction
    MOTOR_MODE_BRAKE,   // Motor is actively braking (if driver supports it)
} motor_mode_t;

// Structure to hold a command for a single motor.
typedef struct {
    motor_mode_t mode;
    uint8_t duty_cycle; // 0-255 for 8-bit PWM resolution
} motor_command_t;

// Initializes the motor driver hardware (PWM, DIR pins).
void s2_init_actuators(void);

// Set commands for both motors.
void s2_set_motor_a(motor_command_t cmd);
void s2_set_motor_b(motor_command_t cmd);

// Emergency stop for all motors.
void s2_emergency_stop(void);

// Reads the state of all limit switches and other safety inputs.
void s2_get_safety_snap(safety_snap_t *snap);


#endif // S2_ACTUATORS_H
