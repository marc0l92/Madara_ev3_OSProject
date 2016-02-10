#include <stdio.h>
#include "lib/my_ev3lib.h"

#define NORMAL_DC	50
#define TURNING_DC	40
#define NORMAL_SPEED	150
#define TURNING_SPEED	200
int i,a,b;

int main(int argc,char** argv){
	// Init
    printf("init\n");
	init_gui();
	init_motors();
	init_sensor();
	
	if(argc != 2){
        printf("Error argc\n");
        return 1;
	}
	switch(atoi(argv[1])){
	case 1:
		// Loop
		while(1){
			set_motors_DC(NORMAL_DC);
			do{
				update_sensor(SENSOR_US);
				printf("> %d\n", get_sensor_value(SENSOR_US));
			}while(get_sensor_value(SENSOR_US) > 500);
			set_motors_DC(0);
			
			set_motorR_DC(TURNING_DC);
			do{
				update_sensor(SENSOR_US);
				printf("> %d\n", get_sensor_value(SENSOR_US));
		    }while(get_sensor_value(SENSOR_US) < 500);
            sleep(1);
			set_motors_DC(0);
		}
	break;
	case 2:
        // arm up down
		set_motor_arm_position(MOTOR_ARM_UP);
		sleep(5);
		set_motor_arm_position(MOTOR_ARM_DOWN);
	break;
	case 3:
        // US setup
		set_motors_speed(NORMAL_SPEED);
        update_sensor(SENSOR_US);
        printf("> %d\n", get_sensor_value(SENSOR_US));
        sleep(1);
        run_relative(1200);
        while(!command_finish());
	break;
	case 4:
        // square
		set_motors_speed(NORMAL_SPEED);
		run_relative(800);
		while(!command_finish());
		turn_relative(90);
		while(!command_finish());
		run_relative(800);
                while(!command_finish());
		turn_relative(90);
		while(!command_finish());
		run_relative(800);
                while(!command_finish());
		turn_relative(90);
		while(!command_finish());
		run_relative(800);
                while(!command_finish());
		turn_relative(90);
	break;
	case 5:
		set_motors_speed(NORMAL_SPEED);
		turn_relative(-360);
	break;
    case 6:
        // US setup
        set_motors_speed(NORMAL_SPEED);
        update_sensor(SENSOR_US);
        printf("> %d\n", get_sensor_value(SENSOR_US));
        sleep(1);
        run_relative(get_sensor_value(SENSOR_US)*2.1 - 47.4);
        while(!command_finish());
    break;
    case 7:
        printf("init compass\n");
        init_compass();
        printf("calibrate compass\n");
        calibrate_compass(0);
        set_motors_speed(200);
        /*for(i=0;i<N_CAL_POINTS;i++){
            turn_absolute_fb(i*360/N_CAL_POINTS);
            sleep(1);
        }*/
        while(1){
            scanf("%d %d",&a,&b);
            turn_absolute_fb(a,b);   
        }
        
       
    break;
	case 8:
		while(1){
			update_sensor(SENSOR_COMPASS);
			printf("> %d\n", get_sensor_value(SENSOR_COMPASS));
			sleep(1);
		}
		break;
	case 9:
		/*
		while(1){
			set_motor_arm_position(MOTOR_ARM_MIDDLE);
			sleep(2);
			update_sensor(SENSOR_COLOR);
			printf("Sensor: %d\n", get_sensor_value(SENSOR_COLOR));
			if(get_sensor_value(SENSOR_COLOR) == COLOR_RED){
				set_motor_arm_position(MOTOR_ARM_UP);
				return 0;
			}
			set_motor_arm_position(MOTOR_ARM_DOWN);
			sleep(5);
		}*/
		while(!grab_and_check());
	}
	return 0;
}
