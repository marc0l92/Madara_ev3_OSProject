#ifndef EV3LIB_MOTOR_CONTROLLER_H
#define EV3LIB_MOTOR_CONTROLLER_H

#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

#include "lib/ev3lib_gui.h"
#include "lib/ev3lib_motor.h"
#include "lib/ev3lib_sensor.h"

#define BLOCKING 0
#define NO_BLOCKING 1

#define GYRO_OFFSET 32767
#define ERROR 1
#define K 0.25
#define Ki 0.0

#define COMPASS_CALIBRATION_SPEED 80
#define N_CAL_POINTS 8
#define N_LOCATIONS 4
#define ERROR_turn_absolute_fb 1
#define COMPASS_CAL_FILE "compass_cal"

void init_compass();
int get_heading_raw();
void calibrate_compass(int new_calibration);
void calibrate_location(char *file_name);
void turn_absolute_fb(int theta_ref,int cal_point);



void turn_relative_fb(int delta_theta);
void stop_fb();

#endif
