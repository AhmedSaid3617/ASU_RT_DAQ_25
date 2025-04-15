#ifndef CONFIG_H
#define CONFIG_H

/* ======================================= TRAVEL ======================================= */
#define LINEAR_POT_PEAK 100

#define CONFIG_TRAVEL_SENSOR_NUM 4
#define CONFIG_ADC_MAX_VALUE 4095

#define CONFIG_PRESSURE_SENSOR_NUM 2

#define CONFIG_TRAVEL_TASK_WAIT 500
/* ===================================== END TRAVEL ===================================== */

/* ===================================== PROXIMITY ===================================== */
#define CONFIG_PROXIMITY_TIMER_FREQ 93632.959
#define CONFIG_TIRE_CIRCUMFERENCE 3.1415 * 0.46482 // 0.46482 is the diameter of the wheel in meters.
/* =================================== END PROXIMITY =================================== */

/* ===================================== IMU ===================================== */
#define CONFIG_IMU_ANGLES_ACCURACY 100.0
#define CONFIG_IMU_ACCELERATION_ACCURACY 100.0
/* =================================== END IMU =================================== */



#endif