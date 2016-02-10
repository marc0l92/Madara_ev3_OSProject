#ifndef EV3LIB_MOTOR_H
#define EV3LIB_MOTOR_H

#include <stdio.h>
#include <stdint.h>
#include "ev3c.h"

#define MOTOR_R_PORT	'C'
#define MOTOR_L_PORT	'B'
#define MOTOR_ARM_PORT	'D'

#define MOTOR_ARM_UP		1
#define MOTOR_ARM_MIDDLE	2
#define MOTOR_ARM_DOWN		3

#define MOTOR_ARM_DC	70
#define MOTOR_ARM_UP_POSITION		-900
#define MOTOR_ARM_MIDDLE_POSITION	-700
#define MOTOR_ARM_DOWN_POSITION		0

#define MOTORS_RAMP_UP		2000
#define MOTORS_RAMP_DOWN	2000

#define MOTOR_TURN_FACTOR 2.08

#define MOTOR_RAMP 0

void init_motors();

void set_motors_DC(uint8_t dc);
void set_motorR_DC(uint8_t dc);
void set_motorL_DC(uint8_t dc);
int32_t get_motorR_position();

void set_motors_speed(int32_t dc);
void set_motorR_speed(int32_t dc);
void set_motorL_speed(int32_t dc);

void set_motors_run_direct();
int32_t command_finish();

void turn_relative(int32_t degree); 
void run_relative(int32_t degree);

void set_motor_arm_position(uint8_t position);

void reset_motors();
void destroy_motors();

#endif
