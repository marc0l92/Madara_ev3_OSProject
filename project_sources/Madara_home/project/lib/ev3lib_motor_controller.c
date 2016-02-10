#include "ev3lib_motor_controller.h"

/*** COMPASS AND TURN ABSOLUTE ***/

int stop_turn_absolute_fb = 0;
int arena_heading,heading;
int references[N_LOCATIONS][N_CAL_POINTS];

/**
*	Store the value of compass when the game starts
**/
void init_compass(){
    update_sensor(SENSOR_COMPASS);
    arena_heading = get_sensor_value(SENSOR_COMPASS);
    printf("[INIT_CONPASS] Arena heading: %d\n",arena_heading);
}

/**
*	Read sensor and convert from [0, 179] to [0, 359]
*	and subtrack initial sensor value
**/
int get_compass_value(){
    int t;
	// Read sensor
    update_sensor(SENSOR_COMPASS);
	// Convert value
    t = ((get_sensor_value(SENSOR_COMPASS) - arena_heading)*2) % 360;
	if(t < 0) t += 360;
    //printf("[GET_COMPASS_VALUE] Sensor value: %d; Converted value: %d\n", get_sensor_value(SENSOR_COMPASS), t);
    return t;
}


void calibrate_location(char *file_name){
	 int i,k;
     FILE *fp;
    
        if((fp=fopen(file_name,"w"))==NULL){
            fprintf(stderr,"Error, could not open %s\n",file_name);
            exit(EXIT_FAILURE);
        }
        printf("\n*****************\n");
        for(i=0;i<N_CAL_POINTS;i++){
            printf("Bring the robot to position %d and press a key\n", 360*i/N_CAL_POINTS);
            scanf("%*c");
            sleep(1);
			k = get_compass_value();
			printf("raw heading = %d\n",k);
            fprintf(fp,"%d\n",k);
        
         }
         
         printf("Bring the robot to position 0 and press a key\n");
         scanf("%*c");

         fclose(fp);     
}

void calibrate_compass(int new_calibration){
     /*start_compass_calibration();
     set_motors_speed(COMPASS_CALIBRATION_SPEED);
     //turn_relative_fb(360);
     turn_relative(360);
     while(!command_finish());
     stop_compass_calibration();
     sleep(1);*/
     int i,k;
     FILE *fp;
    
     if(new_calibration){
        if((fp=fopen(COMPASS_CAL_FILE,"w"))==NULL){
            fprintf(stderr,"Error, could not open %s\n",COMPASS_CAL_FILE);
            exit(EXIT_FAILURE);
        }
        for(k=0;k<N_LOCATIONS;k++){
            printf("\n*****************\nBring the robot to calibration location %d\n",k);
            for(i=0;i<N_CAL_POINTS;i++){
                printf("Bring the robot to position %d and press a key\n", 360*i/N_CAL_POINTS);
                scanf("%*c");
                sleep(1);
                references[k][i] = get_compass_value();
				printf("references[%d][%d]=%d\n",k,i,references[k][i]);
                fprintf(fp,"%d\n",references[k][i]);
        
            }
         }
         printf("Bring the robot to position 0 and press a key\n");
         scanf("%*c");

         fclose(fp);
        //sleep(1);
     } else {
         if((fp=fopen(COMPASS_CAL_FILE,"r"))==NULL){
            fprintf(stderr,"Error, could not open %s\n",COMPASS_CAL_FILE);
            exit(EXIT_FAILURE);
         }

         for(k=0;k<N_LOCATIONS;k++){
            for(i=0;i<N_CAL_POINTS;i++){
                fscanf(fp,"%d",&references[k][i]);
            }
         }



         fclose(fp);

     }
     printf("Calibration done\n");
}

/**
*	Turn of an absolute agnle
*		theta_ref	Target angle
*		cal_point	File to use in order to get the references points
**/
void turn_absolute_fb(int theta_ref, int cal_point){
	sleep(1);
	int theta_compass, theta_ref_cal;
    int theta_error = 0;
	int n_max = 20;
	int index_ref;
    // Reset the stop flag
	stop_turn_absolute_fb = 0;
	// Find the index of the closer angle between:
	//	given angle: theta_ref
	//	angles available in the calibration file
	index_ref = (int)(theta_ref*N_CAL_POINTS/360.0 + 0.5);
	theta_ref_cal = references[cal_point][index_ref];
	
    // Get the references value of that angle
    // Get the references value of that angle
	printf("[TURN_ABSOLUTE_FB] Angle requested: %d; Sensor value requested: %d\n", theta_ref, theta_ref_cal);
    theta_compass = get_compass_value();
    
    // Loop
    while(abs(theta_error = theta_ref_cal - theta_compass) > ERROR_turn_absolute_fb && n_max-- > 0){
	    if(theta_error < -180) theta_error += 360; 
        else if(theta_error > 180) theta_error -= 360; 
        
        //previous_error = theta_error;
        //printf("%d %d %d\n", theta_ref, theta_gyro, theta_error);
        if(stop_turn_absolute_fb){
	        turn_relative(0);
			while(!command_finish());
			break;
		}
        printf("[TURN_ABSOLUTE_FB] Angle requested: %d; Sensor value: %d; Difference error: %d\n",theta_ref, theta_compass, theta_error);
		turn_relative(theta_error * 0.25 );
        while(!command_finish()){
		    if(stop_turn_absolute_fb){
                turn_relative(0);
                while(!command_finish());
                break;
            }
		}
        theta_compass = get_compass_value();
    }
    theta_compass = get_compass_value();
    printf("Nearest cal reached\n");
    theta_error = theta_ref-index_ref*360/N_CAL_POINTS;
    printf("Remaining angle %d \n",theta_error);
    //set_motors_speed(500);
	sleep(1);
	turn_relative(theta_error);
}


        
/*** END COMPASS AND TURN ABSOLUTE **/



int stop = 0;
pthread_t tid;

void stop_fb(){
    stop = 1;
}

void turn_relative_fb(int delta_theta){
	uint16_t theta_gyro, theta_ref;
	int theta_error;
	int integral = 0;
    int previous_error = 0;
	stop = 0;
	
	update_sensor(SENSOR_GYRO);
    theta_ref = get_sensor_value(SENSOR_GYRO) + GYRO_OFFSET;

    theta_ref = (theta_ref + delta_theta);

    update_sensor(SENSOR_GYRO);
    theta_gyro = get_sensor_value(SENSOR_GYRO) + GYRO_OFFSET;

    // Loop
    while(abs(theta_error = theta_ref - theta_gyro) > ERROR && theta_error != previous_error){
        integral += theta_error;
	    previous_error = theta_error;
        //printf("%d %d %d\n", theta_ref, theta_gyro, theta_error);
        if(stop){
	        turn_relative(0);
			while(!command_finish());
			break;
		}
		turn_relative(theta_error * K + integral * Ki);
        while(!command_finish()){
		    if(stop){
                turn_relative(0);
                while(!command_finish());
                break;
            }
		}
        update_sensor(SENSOR_GYRO);
        theta_gyro = get_sensor_value(SENSOR_GYRO) + GYRO_OFFSET;
    }
    update_sensor(SENSOR_GYRO);
    theta_gyro = get_sensor_value(SENSOR_GYRO) + GYRO_OFFSET;

    //printf("Controller -> Desired: %d, Obtained: %d\n", theta_ref, theta_gyro);
}

