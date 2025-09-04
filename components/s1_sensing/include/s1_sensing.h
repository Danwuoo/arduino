#ifndef S1_SENSING_H
#define S1_SENSING_H

#include <stdint.h>
#include <stdbool.h>

// This structure will hold all the sensor data, as defined in the blueprint.
typedef struct {
    float lux;
    float rh;
    float temp;
    bool rain;
    float vbat;
    float wind_speed; // Optional
    float current_motor_a; // Optional
    float current_motor_b; // Optional
    uint64_t timestamp;
} sensor_data_t;

// Initializes all sensors.
void s1_init_sensors(void);

// Reads all sensor values and returns them in the data structure.
void s1_read_sensors(sensor_data_t *data);

#endif // S1_SENSING_H
