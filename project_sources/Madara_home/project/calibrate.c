#include <stdio.h>
#include <stdlib.h>
#include "lib/ev3lib_gui.h"
#include "lib/ev3lib_motor.h"
#include "lib/ev3lib_sensor.h"

#define NORMAL_DC	50
#define TURNING_DC	40
#define NORMAL_SPEED	150
#define TURNING_SPEED	200
int i,a,b;

void main(int argc, char**argv){
	if(argc != 2){
		fprintf(stderr,"Error: usage: ./calibrate <file_name>\n");
		exit(EXIT_FAILURE);
	}
	// Init
	printf("init\n");
	init_gui();
	init_motors();
	init_sensor();
	
	printf("+---+---+\n");
	printf("| 2 | 3 |\n");
	printf("+---+---+\n");
	printf("| 0 | 1 |\n");
	printf("+---+---+\n");
	printf("    ^    \n");
	printf("    |    \n");
	printf("Init compass\n");
	init_compass();
	printf("Calibrate compass\n");
	calibrate_location(argv[1]);

	destroy_sensors();
	destroy_motors();
}
 
