#include <stdio.h>
#include <stdlib.h>
#include "lib/ev3lib_gui.h"
#include "lib/ev3lib_motor.h"
#include "lib/ev3lib_sensor.h"

#define NORMAL_SPEED	150
#define NUM_TESTS       5

int do_test(int dis){
    run_relative(dis);
    while(!command_finish());
    sleep(1);
    run_relative(-1*dis);
    while(!command_finish());
    sleep(1);
    update_sensor(SENSOR_US);
    printf("%d\t%d\n", dis, get_sensor_value(SENSOR_US));
    return get_sensor_value(SENSOR_US);
}

int main(int argc,char** argv){
	int mot_value[] = {200, 400, 600, 1000, 1200};
    int us_value[NUM_TESTS];
    int i;

    // Init
	init_gui();
	init_motors();
	init_sensor();
	
    // US setup
    set_motors_speed(NORMAL_SPEED);
    printf("mot\tus\n");
    
    for(i=0; i<NUM_TESTS; i++){
        us_value[i] = do_test(mot_value[i]);
    }
    
    return 0;
}
