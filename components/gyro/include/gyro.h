#ifndef _GYRO_H
#define _GYRO_H

#include "esp_types.h"

//===============================
typedef struct {
    bool ready_for_check;
    uint16_t max_dt_between_frame; // low frequency noise
    uint16_t max_dt_in_frame; // high frequency noise
} gyroscope_state_t;

extern gyroscope_state_t gyroscope_state;

extern void start_gyro();

#endif
