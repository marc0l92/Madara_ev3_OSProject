#include "ev3lib_ball.h"

int grab_and_check(){
	set_motor_arm_position(MOTOR_ARM_MIDDLE);
	sleep(2);
	update_sensor(SENSOR_COLOR);
	printf("[GRAB_AND_CHECK] Sensor: %d\n", get_sensor_value(SENSOR_COLOR));
	if(get_sensor_value(SENSOR_COLOR) == COLOR_RED || get_sensor_value(SENSOR_COLOR) == COLOR_BLUE){
		set_motor_arm_position(MOTOR_ARM_UP);
		return 1;
	}
	set_motor_arm_position(MOTOR_ARM_DOWN);
	sleep(2);
	return 0;
}
