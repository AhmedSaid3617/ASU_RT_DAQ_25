#ifndef PROXIMITY_PRIV
#define PROXIMITY_PRIV

#include "config.h"

typedef enum {
    FRONT_LEFT_BUFF,
    FRONT_RIGHT_BUFF,
    REAR_LEFT_BUFF,
    REAR_RIGHT_BUFF
} Wheel_buffer_t;

#define PROXIMITY_CALCULATE_SPEED(rpm1, rpm2) ((((rpm1 + rpm2) / 2.0) * CONFIG_TIRE_CIRCUMFERENCE * 60) / 1000)

#endif