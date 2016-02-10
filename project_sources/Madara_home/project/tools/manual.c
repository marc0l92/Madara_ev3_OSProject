#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <limits.h>
#include "lib/my_ev3lib.h"

#define NORMAL_SPEED    250
#define SLOW_SPEED      80
#define FORWARD_DISTANCE 30
#define TURNING_DISTANCE 10

int main(){
	char c;
	// Setup periferal and library
    init_gui();
    init_motors();
	
	system ("/bin/stty raw");
	
	set_motors_speed(NORMAL_SPEED);

	while(1){
		c = getchar();
		switch(c){
			case 'q':
				system ("/bin/stty cooked");
				return 0;
				break;
			case 'w':
                run_relative(FORWARD_DISTANCE);
				break;
			case 's':
                run_relative(-1*FORWARD_DISTANCE);
				break;
			case 'a':
				turn_relative(-1*TURNING_DISTANCE);
				break;
			case 'd':
				turn_relative(TURNING_DISTANCE);
				break;
			case '1':
				set_motor_arm_position(MOTOR_ARM_UP);
				break;
			case '2':
				set_motor_arm_position(MOTOR_ARM_DOWN);
				break;
		}
	}

	system ("/bin/stty cooked");
	return 0;
}
