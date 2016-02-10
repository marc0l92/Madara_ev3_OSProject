#ifndef EV3LIB_SENSOR_H
#define EV3LIB_SENSOR_H

#include <stdio.h>
#include "ev3c.h"

#define SENSOR_US_PORT 2
#define SENSOR_GYRO_PORT 3
#define SENSOR_COLOR_PORT 1
#define SENSOR_COMPASS_PORT 4

#define SENSOR_US 1
#define SENSOR_GYRO 2
#define SENSOR_COLOR 3
#define SENSOR_COMPASS 4

#define COLOR_RED	5
#define COLOR_BLUE	2

void init_sensor();

void start_compass_calibration();
void stop_compass_calibration();

void update_sensor(uint8_t id);
int32_t get_sensor_value(uint8_t id);

void destroy_sensor();

#endif
