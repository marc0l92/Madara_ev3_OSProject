#include "ev3lib_motor.h"
#include "ev3lib_gui.h"

ev3_motor_ptr motorL = NULL, motorR = NULL, motorARM = NULL;
ev3_motor_ptr motors_list = NULL;
int ramp_on = 0;

void init_motors(){
	//Loading all motors
	motors_list = ev3_load_motors();
	
	//Init right motor
	motorR = ev3_search_motor_by_port(motors_list, MOTOR_R_PORT);
	if(motorR == NULL){
		error_message("Motor Right not found\n");
	}
	ev3_reset_motor(motorR);
	ev3_open_motor(motorR);
	
	//Init left motor
	motorL = ev3_search_motor_by_port(motors_list, MOTOR_L_PORT);
	if(motorL == NULL){
		error_message("Motor Left not found\n");
	}
	ev3_reset_motor(motorL);
	ev3_open_motor(motorL);

	//Init arm motor
        motorARM = ev3_search_motor_by_port(motors_list, MOTOR_ARM_PORT);
        if(motorARM == NULL){
                error_message("Motor Arm not found\n");
        }
        ev3_reset_motor(motorARM);
        ev3_open_motor(motorARM);
	
	//Set motors stop mode: "coast", "brake", "hold"
	ev3_stop_command_motor_by_name(motorR, "hold");
	ev3_stop_command_motor_by_name(motorL, "hold");
	ev3_stop_command_motor_by_name(motorARM, "hold");

	//Set motors mode
	//set_motors_DC(0);
	//set_motors_speed(0);
	ev3_command_motor_by_name(motorR, "run-direct");
	ev3_command_motor_by_name(motorL, "run-direct");

	//Set arm mode
	ev3_set_duty_cycle_sp(motorARM, MOTOR_ARM_DC);
    set_motor_arm_position(MOTOR_ARM_DOWN);
}

void set_motors_DC(uint8_t dc){
	ev3_set_duty_cycle_sp(motorR, dc);
	ev3_set_duty_cycle_sp(motorL, dc);
}

void set_motorR_DC(uint8_t dc){
	ev3_set_duty_cycle_sp(motorR, dc);
}

void set_motorL_DC(uint8_t dc){
	ev3_set_duty_cycle_sp(motorL, dc);
}

void set_motors_speed(int32_t speed){
	ev3_set_speed_sp(motorR, speed);
	ev3_set_speed_sp(motorL, speed);
}

void set_motorR_speed(int32_t speed){
	ev3_set_speed_sp(motorR, speed);
}

void set_motorL_speed(int32_t speed){
	ev3_set_speed_sp(motorL, speed);
}

int32_t get_motorR_position() {
    return ev3_get_position(motorR);
}

void set_motor_run_direct(){
	ev3_command_motor_by_name(motorR, "run-direct");
	ev3_command_motor_by_name(motorL, "run-direct");
}

int32_t command_finish(){
	return (ev3_motor_state(motorR) & MOTOR_HOLDING && ev3_motor_state(motorL) & MOTOR_HOLDING && ev3_motor_state(motorARM) & MOTOR_HOLDING);
}

void turn_relative(int32_t degree){
#if MOTOR_RAMP
	if(ramp_on){
		ev3_set_ramp_up_sp(motorR, 0);
		ev3_set_ramp_up_sp(motorL, 0);
		ev3_set_ramp_down_sp(motorR, 0);
		ev3_set_ramp_down_sp(motorL, 0);
		ramp_on = 0;
	}
#endif
	ev3_set_position_sp(motorR, -1 * degree * MOTOR_TURN_FACTOR);
	ev3_set_position_sp(motorL, degree * MOTOR_TURN_FACTOR);
	ev3_command_motor_by_name(motorR, "run-to-rel-pos");
    ev3_command_motor_by_name(motorL, "run-to-rel-pos");
}

void run_relative(int32_t degree){
#if MOTOR_RAMP
	if(!ramp_on){
		ev3_set_ramp_up_sp(motorR, MOTORS_RAMP_UP);
		ev3_set_ramp_up_sp(motorL, MOTORS_RAMP_UP);
		ev3_set_ramp_down_sp(motorR, MOTORS_RAMP_DOWN);
		ev3_set_ramp_down_sp(motorL, MOTORS_RAMP_DOWN);
		ramp_on = 1;
	}
#endif
	ev3_set_position_sp(motorR, degree);
	ev3_set_position_sp(motorL, degree);
	ev3_command_motor_by_name(motorR, "run-to-rel-pos");
    ev3_command_motor_by_name(motorL, "run-to-rel-pos");
}

void set_motor_arm_position(uint8_t position){
	switch(position){
		case MOTOR_ARM_DOWN:
			ev3_set_position_sp(motorARM, MOTOR_ARM_DOWN_POSITION);
			ev3_command_motor_by_name(motorARM, "run-to-abs-pos");
			break;
		case MOTOR_ARM_MIDDLE:
			ev3_set_position_sp(motorARM, MOTOR_ARM_MIDDLE_POSITION);
			ev3_command_motor_by_name(motorARM, "run-to-abs-pos");
			break;
		case MOTOR_ARM_UP:
			ev3_set_position_sp(motorARM, MOTOR_ARM_UP_POSITION);
			ev3_command_motor_by_name(motorARM, "run-to-abs-pos");
			break;
	}
}

void reset_motors(){
    ev3_reset_motor(motorR);
    ev3_reset_motor(motorL);
    ev3_reset_motor(motorARM);
}

void destroy_motors(){
	if(motors_list != NULL)
		ev3_delete_motors(motors_list);
}
