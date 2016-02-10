#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <limits.h>
#include "lib/my_ev3lib.h"

// 0 = Big arena; 1 = Small arena
#define ARENA 1
#if ARENA == 0
	#define POINTS_FILE_NAME "leader_points_big"
	#define BORDER_X_MAX 200
	#define BORDER_Y_MAX 400
#else
	#define POINTS_FILE_NAME "leader_points_small"
	#define BORDER_X_MAX 120
	#define BORDER_Y_MAX 200
#endif

#define WALL 300

#define STOP_DIST 200

#define NORMAL_SPEED    250
#define SLOW_SPEED      80

#define SENSOR_US_THRESHOLD 80
#define BORDER_THRESHOLD 10

#define CUSTOM_STOP_MESSAGE 0
#define REACHED 1
#define OBSTACLE 2

typedef struct _pos_t{
	int32_t x;
	int32_t y;
	int32_t t; // Tetha
	int32_t d;
} pos_t;

typedef struct _act_val_t{
	int angle;
	int dist;
	int speed;
} act_val_t;

act_val_t act_val;

int robot_rank, snake_size, prev=255, next=255;
pos_t robot;
pos_t min;
pos_t home,center;
int gyro_init_val = 0;

int n_points;
pos_t *positions;

int send_action_flag = 1;

int act = 0;
int end_game = 0;
int start_game = 0;
int wait = 0;
int cancel = 0;

int get_cal_location(int x,int y){
	int i=0,j=0;
	if(x > BORDER_X_MAX/2) i=1;
	if(y > BORDER_Y_MAX/2) j=1;
	printf("cal loc %d\n",2*j+i);
	return 2*j+i;
}

int get_gyro_value(){
	update_sensor(SENSOR_GYRO);
	return (get_sensor_value(SENSOR_GYRO) - gyro_init_val);
}

int check_ball(uint32_t d, uint32_t t){
	int found = 0; 
	set_light(LIT_LEFT, LIT_OFF);
	if(d < WALL){ 
		if( d < min.d){
			min.d = d;
			min.t = t;
			printf("Distance: %d Angle: %d NEW MIN!\n", d);
			set_light(LIT_LEFT, LIT_RED);
			found = 1;
		}
	}
	return found;
}

int scan_for_ball(){
	int dist;
	int found=0;
	min.d =2500;

    set_light(LIT_RIGHT, LIT_GREEN);

    set_motors_speed(SLOW_SPEED);
	turn_relative(90);
	while(!command_finish()){   
        update_sensor(SENSOR_US);
		found |= check_ball(get_sensor_value(SENSOR_US), get_gyro_value());
    }
	//update robot angle
	robot.t += 90;
	
    turn_relative(-180);
    while(!command_finish()){
        update_sensor(SENSOR_US);
        found |= check_ball(get_sensor_value(SENSOR_US), get_gyro_value());
    }
	
	//update robot angle
	robot.t -= 180;

    if(found == 1){
        // Durante la scansione ho trovato un minimo
        sleep(1);
        printf("Start: %d -> Stop teorico: %d, Distanza teorica: %d\n", get_gyro_value(), min.t, min.d);
        turn_relative_fb(min.t-get_gyro_value());
		
		//update robot angle
		robot.t = get_gyro_value();
        printf("Stop reale: %d, Distanza reale: %d\n", robot.t, dist);
    }else{
        // Durante la scansione non ho trovato minimi
        turn_relative_fb(90);
        //while(!command_finish());
		
		//update robot angle
		robot.t = get_gyro_value();
    }
    set_light(LIT_RIGHT, LIT_OFF);
    return found;
}

void drop_the_ball(){
    set_motors_speed(NORMAL_SPEED);
    set_motor_arm_position(MOTOR_ARM_DOWN);
    while(!command_finish());
    sleep(5);
}

int get_ball(){
	int tries = 2; //number of tries when scanning to find the ball
	int found = 0;
    int angle;
	int dist;
	while(tries > 0){
	    found = scan_for_ball();
    	if(found == 1){
            // Min point found
            if(min.d <120){
                // Grab the ball
                set_motors_speed(NORMAL_SPEED);
				angle = min.d * 2.1 - 67;
                run_relative(angle);
                while(!command_finish());
				dist = (angle*18.0/360);
				robot.x += dist * sin(robot.t);
				robot.y += dist * cos(robot.t);
				//add check that the ball is really there
                tries = (grab_and_check())? 0: 2;
            }else{
                // Avvicinati
                set_motors_speed(NORMAL_SPEED);
				angle = (int)((min.d * 2.1 -67)*3/4);
                run_relative(angle);
				dist = (angle*18.0/360);
				robot.x += dist * sin(robot.t);
				robot.y += dist * cos(robot.t);
                while(!command_finish());
            }
        }else{
              // Minimum not found
			  //set_motors_speed(NORMAL_SPEED);
              //run_relative(0);
              //while(!command_finish());
              tries--;
        }
    }

    set_light(LIT_LEFT, LIT_OFF);
    set_light(LIT_RIGHT, LIT_OFF);
	
	//update robot angle
	robot.t = get_gyro_value();
    return found;    
}

void compute_polar_coord(pos_t target_pos, int *dist, int *angle, int margin){
	double arg;

	arg=1.0*(target_pos.y - robot.y)/(target_pos.x - robot.x);
	printf("goal: %d %d\n",target_pos.x,target_pos.y);
     
	if((target_pos.x - robot.x)<0)
		*angle=(int)(90-(atan(arg)*180/M_PI))+180;//-180;
	else
		*angle= (int)(90-(atan(arg)*180/M_PI));
        
	printf("arg :%lf\n",arg);
	printf("arco: %lf\n",atan(arg)*180/M_PI);
	printf("angle: %d\n", *angle);
	//calcolo la distanza da percorrere
	*dist=(int)sqrt((target_pos.x - robot.x)*(target_pos.x - robot.x) + (target_pos.y - robot.y)*(target_pos.y - robot.y))-margin;
	printf("dist: %d\n", *dist);
}

//this function makes the robot avoid the obstacle running for a small distance in another direction
//this direction depends on the current angle and position
void obstacle_avoid(){
	int quadrant = robot.t / 90;
	int right_left_n = robot.x / BORDER_X_MAX/2;
	int up_down_n = robot.y / BORDER_Y_MAX/2;
	int angles[4][2][2];

	//first quadrant
	angles[0][0][0]=45; //down left
	angles[0][0][1]=-45; //down right
	angles[0][1][0]=45; //up left
	angles[0][1][1]=-45; //up right
	
	//second quadrant
	angles[1][0][0]=-45; //down left
	angles[1][0][1]=-45; //down right
	angles[1][1][0]=45; //up left
	angles[1][1][1]=45; //up right
	
	//third quadrant
	angles[2][0][0]=-45; //down left
	angles[2][0][1]=45; //down right
	angles[2][1][0]=-45; //up left
	angles[2][1][1]=45; //up right
	
	//fourth quadrant
	angles[3][0][0]=45; //down left
	angles[3][0][1]=45; //down right
	angles[3][1][0]=-45; //up left
	angles[3][1][1]=-45; //up right

	int angle=(robot.t+angles[quadrant][up_down_n][right_left_n])%360;

    if(robot_rank ==0 && send_action_flag) send_action(next,angle,40,NORMAL_SPEED/2);

    set_motors_speed(NORMAL_SPEED);
	turn_absolute_fb(angle,get_cal_location(robot.x,robot.y));
    robot.t=angle;
    run_relative((int)40*360.0/18);
    while(!command_finish());
    robot.x= robot.x + 40*sin(robot.t*M_PI/180);
    robot.y= robot.y + 40*cos(robot.t*M_PI/180);

	printf("\n\n>>>>>>>>pos %d %d %d\n\n",robot.x,robot.y,robot.t);
}

//return 1 if obstacle found
int obstacle(){
	update_sensor(SENSOR_US);
	if(get_sensor_value(SENSOR_US) < STOP_DIST) {
		return 1;
	}
	return 0;
}

//return REACHED if position reached, OBSTACLE if stopped because of obstacle A CUSTOM STOP MESSAGE MUST BE SENT, CUSTOM_STOP_MESSAGE if stopped because a custom message was sent
//robot position is updated
//position is reached running for distance in steps <=200 (first the dist%200 than n steps dist/200)
//it sleeps 1s after each step
int go_to_position(pos_t target_pos, int margin, int speed){
	int32_t tacho_dist, start_tacho, end_tacho;
	int angle, dist;
	int i;
     
	if(robot_rank == 0) while(wait);

	compute_polar_coord(target_pos, &dist, &angle,margin);
	target_pos.x = robot.x + dist*sin(angle*M_PI/180);
	target_pos.y = robot.y + dist*cos(angle*M_PI/180);

    //start movement, in two steps if bigger than 200
    int div = dist/200;
	int mod = dist%200;
	int step;
	for(i = 0; i < div+1 ; i++){
		if(i == div) step = mod;
		else step = 200;

        if(robot_rank == 0 && send_action_flag) send_action(next, angle, step, speed/2);
        //vado alla posizione
        set_motors_speed(SLOW_SPEED);
        turn_absolute_fb(angle, get_cal_location(robot.x,robot.y));
        //aggiorno posizione final;
        robot.t = angle;//get_gyro_value();
        printf("robot.t = %d\n",robot.t);
		
		//save initial tacho counts
        start_tacho=get_motorR_position();
        set_motors_speed(speed);
        run_relative((int)(step*360.0/18));
        while(!command_finish()){
			if(cancel){
				//cancel = 0;
				run_relative(0);
				end_tacho = get_motorR_position();
				tacho_dist = end_tacho-start_tacho;
				dist = tacho_dist/20.0;
				robot.x = robot.x + dist*sin(robot.t*M_PI/180);
				robot.y = robot.y + dist*cos(robot.t*M_PI/180);
				printf("\n\n>>>>>>>>pos %d %d %d\n\n",robot.x,robot.y,robot.t);
				if(send_action_flag) send_cancel(next, dist);
				// Send to ourself an action in order came closer
				act_val.dist = cancel - dist;
				act_val.angle = robot.t;
				act++;
				return CUSTOM_STOP_MESSAGE;

			}

			if(obstacle()){
				run_relative(0);
				end_tacho = get_motorR_position();
				tacho_dist = end_tacho-start_tacho;
				dist = tacho_dist/20.0;
				printf("step %d dist %d\n",step,dist);
				if(step-dist < STOP_DIST*18/360.0){
					set_motors_speed(speed);
					run_relative((int)((step-dist)*360.0/18));
					return REACHED;
				}
				printf("tacho dist %ld dist %ld\n",tacho_dist,dist);
				robot.x = robot.x + dist*sin(robot.t*M_PI/180);
				robot.y = robot.y + dist*cos(robot.t*M_PI/180);
				printf("\n\n>>>>>>>>pos %d %d %d\n\n",robot.x,robot.y,robot.t);

				if(send_action_flag) send_cancel(next,dist);
				// HERE WE MUST SEND A CUSTOM MESSAGE STOP TO FOLLOWING ROBOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				// HERE WE MUST SEND A CUSTOM MESSAGE STOP TO FOLLOWING ROBOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				return OBSTACLE;
			}
		}
		sleep(1);
	}
	
	//update pos
	robot.x = target_pos.x; 
	robot.y = target_pos.y;

	printf("\n\n>>>>>>>>pos %d %d %d\n\n",robot.x,robot.y,robot.t);
	return REACHED;  
}

//this function tris to reach a position avoiding an obstacle and restarting every time an obstacle is found
//may deadlock... even if care is taken to properly avoid obsatcles
void go_to_position_interruptible(pos_t target_pos, int margin, int speed, int avoid_obstacle) {
    int result;
    while(1) {
		result = go_to_position(target_pos, margin, speed);
		if(result == REACHED || result == CUSTOM_STOP_MESSAGE) break;
		if(result == OBSTACLE){
			if(avoid_obstacle){
				obstacle_avoid();
			}else{
				break;
			}
		}
    }
}

void get_positions(char* filename, pos_t ** positions, int *n_points ){
    int i;
    FILE * fp = fopen(filename,"r");
    if(fp == NULL){
        fprintf(stderr,"Error opening file %s\n",filename);
        exit(EXIT_FAILURE);
    }

    fscanf(fp,"%d",n_points);
    printf("N points = %d\n",*n_points);
    *positions = (pos_t*)malloc((*n_points)*sizeof(pos_t));
    if(positions == NULL){
        fprintf(stderr,"Error in malloc\n");
        exit(EXIT_FAILURE);
    }

    for(i=0;i<*n_points;i++){
        fscanf(fp,"%d %d",&((*positions)[i].x),&((*positions)[i].y));
        printf("point %d) %d %d\n",i, (*positions)[i].x,(*positions)[i].y);
    }

    fclose(fp);
}

int leader(){
    int i=0;
    int found = 0;
    while(!found){
        if(go_to_position(positions[i], 10, NORMAL_SPEED)){
			//predefined posiion reached
			found = get_ball();
			//if(found){ //decide if and when reset  positions to prefdefined one
				robot.x = positions[i].x;
				robot.y = positions[i].y;
				i++;
			//}
		} else {
			//obstacle or ball found... must chose if it is to avoid or to grab
			//for now assume it is a ball
			int a = robot.t;
			found = get_ball();
			robot.x+=sin(a*M_PI/180)*STOP_DIST;
			robot.y+=cos(a*M_PI/180)*STOP_DIST;
		}
        
        
        if(i==n_points) i = 0;
    }
}

/*void update_robot_pos(int angle,int dist){
    robot.t = angle; //printf("angle %d dist %d\n",angle,dist);
    robot.x += dist*sin(angle*M_PI/180.0);
    robot.y += dist*cos(angle*M_PI/180.0);
    printf("Current position %d %d %d\n",robot.x,robot.y,robot.t);
}*/

void follower_action_cb(uint8_t src, uint16_t angle, uint8_t dist, uint16_t speed){
	while(act > 0);
	act_val.dist = dist;
	act_val.angle = angle;
	act_val.speed = speed;
	act = 1;
}

void follower(){
	while(act <= 0);

    pos_t leader_position;
    leader_position.x = robot.x + 40*sin(robot.t*M_PI/180.0) + act_val.dist*sin(act_val.angle*M_PI/180.0);
    leader_position.y = robot.y + 40*cos(robot.t*M_PI/180.0) + act_val.dist*cos(act_val.angle*M_PI/180.0);

    if(robot_rank >0){
        printf("[STATUS] Action received, starting to follow\n");
        if(act_val.dist < 40 || (act_val.angle > robot.t + 90 && act_val.angle < robot.t + 270 )){
            go_to_position_interruptible(leader_position, 40, 2*act_val.speed, 0) ; //now it doers not avoid obstacles
        }else{
			//must be done with go_to_postion_interruptible too...
			leader_position.x = robot.x + 40*sin(robot.t*M_PI/180.0);
			leader_position.y = robot.y + 40*cos(robot.t*M_PI/180.0);
			//printf("***************************** %d %d\n",robot.x,robot.y);
			go_to_position_interruptible(leader_position,0,2*act_val.speed,0);
			//printf("****************************** %d %d\n",robot.x,robot.y);
			if(!cancel){
				leader_position.x = robot.x + (act_val.dist-40)*sin(act_val.angle*M_PI/180.0);
				leader_position.y = robot.y + (act_val.dist-40)*cos(act_val.angle*M_PI/180.0);
			    go_to_position_interruptible(leader_position, 0, 2*act_val.speed, 0);
			
				set_motors_speed(NORMAL_SPEED);
				turn_absolute_fb(act_val.angle, get_cal_location(robot.x,robot.y));
				robot.t = act_val.angle;
			}
		}
	}
	cancel = 0;
	act--;
}

void start_cb(uint8_t rank, uint8_t size, uint8_t p, uint8_t n){
    start_game = 1;
    robot_rank = rank;
    snake_size = size;
    prev = p;
    next = n;
}

//to be sure remember to check in the library if we are the recepients
void lead_cb(){
    robot_rank = 0;
}

void wait_cb(uint8_t src, uint8_t delay){
    wait = 1;
    sleep(delay);
    wait = 0;
}

void sigint_handler(){
	// Reinstall handler (for portability)
	signal(SIGINT,sigint_handler);
	printf("\n\n************************\nSIGINT received\nExiting gently\n\n");
	destroy_sensors();
	destroy_motors();
	destroy_bluetooth();
	free(positions);
	exit(EXIT_FAILURE);
}

void stop_cb(){
    end_game = 1;
	sigint_handler(); // in this case end_game is useless
}

void cancel_cb(uint8_t dist){
    // Set the cancel flag to the distance received
	if(dist == 0xFF || dist == 0){
		cancel = 1;
	}else{
		cancel = dist;
	}
}

void init(int use_http, char *address, int port){
    // Setup periferal and library
    init_gui();
    init_motors();
    init_sensor();
	init_compass();
	calibrate_compass(0); // 0 = do not execute a new calibration of the compass	

	// Load ball predefined positions
	get_positions(POINTS_FILE_NAME, &positions, &n_points);

	// Start bluetooth
	if(use_http)
		bluetooth_test_init(address, port);
    else 
		bluetooth_init();
	
	//bluetooth_register_and_start(fAction_t, fAck_t, fLead_t, fStart_t, fStop_t, fWait_t, fKick_t, fCancel_t);
    bluetooth_register_and_start(&follower_action_cb, NULL, &lead_cb, &start_cb, &stop_cb, NULL, NULL, &cancel_cb);

	// Reset global variables
  	send_action_flag = 1;
	act = 0;
    end_game = 0;
    start_game = 0;
    wait = 0;
	cancel = 0;
	// Waiting for the game start
	set_light(LIT_LEFT, LIT_GREEN);
	set_light(LIT_RIGHT, LIT_GREEN);
    while(!start_game); //start game, robot rank and snake size  initialized by start_cb

	// Initial position
	home.x = BORDER_X_MAX/2;
	home.y = (snake_size - robot_rank -1)*40 + 20;
	center.x = BORDER_X_MAX/4;
	center.y = BORDER_Y_MAX/2;
	robot = home;
	
	printf("[DEBUG] Starting position X: %d, Y: %d, T: %d\n", robot.x , robot.y, robot.t);
	
	update_sensor(SENSOR_GYRO);
    gyro_init_val = get_sensor_value(SENSOR_GYRO);
    set_light(LIT_LEFT, LIT_OFF);
	set_light(LIT_RIGHT, LIT_OFF);
}

int go_back_home(){
	send_action_flag = 0;
    printf("[STATUS] I am going back home\n");
    //go_to_position_interruptible(center, 0, NORMAL_SPEED, 1);
    //turn_absolute_fb(180, get_cal_location(robot.x, robot.y));
	pos_t drop;
	drop.x = BORDER_X_MAX/2;
	drop.y = 20;
    go_to_position_interruptible(drop, 0, NORMAL_SPEED, 1);
    turn_absolute_fb(180, get_cal_location(robot.x, robot.y));

    drop_the_ball();
    return 1;
}

int main(int argc,char** argv){
		int found = 0, arrived_home = 0;
		int use_http = 0;
		char address[200];
		int port = 55555;
		strcpy(address, "localhost");
    
		if(argc >= 4){
            use_http = atoi(argv[1]);
            strncpy(address, argv[2], 200);
            port = atoi(argv[3]);
		}else{
            printf("[INFO] Usage: 'prog 1 address port' to start it with http connection\n");
		}

	    //initalization
		init(use_http, address, port);
	  
		//install signal handler to exit gently
		signal(SIGINT,sigint_handler);
    
		//scheduling
		while(!end_game && !arrived_home){
			if(robot_rank == 0){
				// Leader mode
				if(found == 0){ 
					found = leader();
                }else{
                    arrived_home = go_back_home();
                    send_lead(next);
                }
            }else{
                 // Follower mode
                 while(robot_rank > 0 && !end_game){
					follower();
				 }//######################aggiunger end game e stop anche nel leader e go back home etc
                 //follower call back called at each action messagge
            }
		}

		if(end_game){
            printf("[STATUS] Game ended\n");
		}else{
            printf("[STATUS] Back home\n");
		}
       
		reset_motors();
		destroy_sensors(); 
		destroy_motors();
		destroy_bluetooth();
		free(positions);
}
