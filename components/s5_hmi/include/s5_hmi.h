#ifndef S5_HMI_H
#define S5_HMI_H

#include "s3_state_machine.h"
#include "s1_sensing.h"

// Initializes the HMI components (LCD, buttons, LEDs).
void s5_init_hmi(void);

// Updates the display with the latest data.
void s5_update_display(system_state_t state, const sensor_data_t *sensors);

// Checks for button presses.
void s5_check_buttons(void);

#endif // S5_HMI_H
