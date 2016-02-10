#include "ev3lib_bluetooth.h"
#include <arpa/inet.h>

#define DELAY_ON_ACK 1

int bt, test_mode=0;
uint8_t buff_read[BUFFER_SIZE], buff_write[BUFFER_SIZE];
uint16_t message_id = 0;

fAction_t fAction;
fAck_t fAck;
fLead_t fLead;
fStart_t fStart;
fStop_t fStop;
fWait_t fWait;
fKick_t fKick;
fCancel_t fCancel;

pthread_t bluetooth_thread;

void bluetooth_init(){
	struct sockaddr_rc addr = {0};
	// Create socket
    test_mode = 0;
	bt = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	// Setup connection
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba(SERV_ADDRESS, &addr.rc_bdaddr);
	
	// start connection
	if(connect(bt, (struct sockaddr *)&addr, sizeof(addr)) != 0){
		// Connection fail
        printf("Bluetooth connection fail\n");
		//error_message("Bluetooth connection fail");
	}
}

void bluetooth_test_init(char *ip_addr, int port){
    struct sockaddr_in addr;
    // Create socket
    test_mode = 1;
    bt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    // Setup connection
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    struct in_addr address;
    inet_aton(ip_addr, &address);
    addr.sin_addr = address;

    // start connection
    if(connect(bt, (struct sockaddr *)&addr, sizeof(addr)) != 0){
        // Connection fail
        printf("Bluetooth connection fail\n");
        //error_message("Bluetooth connection fail");
    }
}

int bluetooth_read(){
    int bytes_read;
    if (test_mode){
        bytes_read = recv(bt, buff_read, BUFFER_SIZE, 0);
    }else{
	    bytes_read = read(bt, buff_read, BUFFER_SIZE);
    }
	if(bytes_read <= 0) {
		close(bt);
		printf("Bluetooth connection closed\n");
        //error_message("Bluetooth connection closed");
	}
	return bytes_read;
}

int bluetooth_send(int size){
    if (test_mode){
        return send(bt, buff_write, size, 0);
    }else{
	    return write(bt, buff_write, size);
    }
}

/*
    0       1       2       3       4       5       6       7       8       9
+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
|      ID       |  src  |  dst  |   0   |     angle     | dist  |     speed     |
+-------+-------+-------+-------+-------+-------+-------+-------+-------+-------+
*/
int send_action(uint8_t dst, uint16_t angle, uint8_t dist, uint16_t speed){
    printf("+ Send action\n");
	if(dst != 255){
		buff_write[0] = message_id & 0xFF;
		buff_write[1] = message_id >> 8;
		buff_write[2] = TEAM_ID;
		buff_write[3] = dst;
		buff_write[4] = MSG_ACTION;
		buff_write[5] = angle & 0xFF;
		buff_write[6] = angle >> 8;
		buff_write[7] = dist;
		buff_write[8] = speed & 0xFF;
		buff_write[9] = speed >> 8;
		message_id++;
		return bluetooth_send(10);
	}else{
		return 0;
	}
}

/*
    0       1       2       3       4       5       6       7
+-------+-------+-------+-------+-------+-------+-------+-------+
|      ID       |  src  |  dst  |   1   |    ID ack     | state |
+-------+-------+-------+-------+-------+-------+-------+-------+
state: 0 -> OK, 1 -> error
*/
int send_ack(uint8_t dst, uint16_t msg_id, uint8_t state){
    printf("+ Send ack\n");
	if(dst != 255){
		buff_write[0] = message_id & 0xFF;
		buff_write[1] = message_id >> 8;
		buff_write[2] = TEAM_ID;
		buff_write[3] = dst;
		buff_write[4] = MSG_ACK;
		buff_write[5] = msg_id & 0xFF;
		buff_write[6] = msg_id >> 8;
		buff_write[7] = state;
		message_id++;
		return bluetooth_send(8);
	}else{
		return 0;
	}
}

/*
    0       1       2       3       4
+-------+-------+-------+-------+-------+
|      ID       |  src  |  dst  |   2   |
+-------+-------+-------+-------+-------+
*/
int send_lead(uint8_t dst){
    printf("+ Send lead\n");
	if(dst != 255){
		buff_write[0] = message_id & 0xFF;
		buff_write[1] = message_id >> 8;
		buff_write[2] = TEAM_ID;
		buff_write[3] = dst;
		buff_write[4] = MSG_LEAD;
		message_id++;
		return bluetooth_send(5);
	}else{
		return 0;
	}
}

/*
    0       1       2       3       4       5
+-------+-------+-------+-------+-------+-------+
|      ID       |  src  |  dst  |   5   | delay |
+-------+-------+-------+-------+-------+-------+
*/
int send_wait(uint8_t dst, uint16_t delay){
    printf("+ Send wait\n");
	buff_write[0] = message_id & 0xFF;
	buff_write[1] = message_id >> 8;
	buff_write[2] = TEAM_ID;
	buff_write[3] = dst;
	buff_write[4] = MSG_WAIT;
	buff_write[5] = delay;
	message_id++;
	return bluetooth_send(10);
}

/*
    0       1       2       3       4       5
+-------+-------+-------+-------+-------+-------+
|      ID       |  src  |  dst  |   5   | delay |
+-------+-------+-------+-------+-------+-------+
*/
int send_cancel(uint8_t dst, uint8_t dist){
    printf("+ Send cancel\n");
	if(dst != 255){
		buff_write[0] = message_id & 0xFF;
		buff_write[1] = message_id >> 8;
		buff_write[2] = TEAM_ID;
		buff_write[3] = dst;
		buff_write[4] = MSG_CANCEL;
		buff_write[5] = dist;
		message_id++;
		return bluetooth_send(10);
	}else{
		return 0;
	}
}

void *bluetooth_read_loop(void *x){
	while(bluetooth_read() > 0){
		switch(buff_read[4]){
			case MSG_ACTION:
                printf("+ Received Action\n");
				send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
                sleep(DELAY_ON_ACK);
                printf("+ Do Action\n");
				if(fAction != NULL && buff_read[3] == TEAM_ID){
					fAction(
						buff_read[2],
						buff_read[5] + (buff_read[6]<<8),
						buff_read[7],
						buff_read[8] + (buff_read[9]<<8));
				}
				break;
			case MSG_ACK:
                printf("+ Received Ack\n");
				if(fAck != NULL && buff_read[3] == TEAM_ID){
					fAck(
						buff_read[2],
						buff_read[5] + (buff_read[6]<<8),
						buff_read[7]);
				}
				break;
			case MSG_LEAD:
                printf("+ Received Lead\n");
				send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
                sleep(DELAY_ON_ACK);
				if(fLead != NULL && buff_read[3] == TEAM_ID){
					fLead();
				}
				break;
			case MSG_START:
                printf("+ Received Start\n");
				//send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
				if(fStart != NULL && buff_read[3] == TEAM_ID){
					fStart(
						buff_read[5],
						buff_read[6],
						buff_read[7],
						buff_read[8]);
				}
				break;
			case MSG_STOP:
                printf("+ Received Stop\n");
				//send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
                //sleep(DELAY_ON_ACK);
				if(fStop != NULL && buff_read[3] == TEAM_ID){
					fStop();
				}
				break;
			case MSG_WAIT:
				send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
                sleep(DELAY_ON_ACK);
				if(fWait != NULL && buff_read[3] == TEAM_ID){
					fWait(
						buff_read[2],
						buff_read[5]);
				}
				break;
			case MSG_CUSTOM:
				printf("+ Custom message received\n");
				break;
			case MSG_KICK:
				printf("+ Received Kick\n");
				if(fKick != NULL && buff_read[3] == TEAM_ID){
					fKick(buff_read[5], buff_read[6], buff_read[7]);
				}
				break;
			case MSG_CANCEL:
				printf("+ Received Cancel\n");
				send_ack(buff_read[2], buff_read[0]+(buff_read[1]<<8), 0);
                sleep(DELAY_ON_ACK);
				if(fCancel != NULL && buff_read[3] == TEAM_ID){
					fCancel(buff_read[5]);
				}
				break;
		}
	}
    return NULL;
}

int bluetooth_register_and_start(fAction_t action, fAck_t ack, fLead_t lead, fStart_t start, fStop_t stop, fWait_t wait, fKick_t kick, fCancel_t cancel){
	fAction = action;
	fAck = ack;
	fLead = lead;
	fStart = start;
	fStop = stop;
	fWait = wait;
	fKick = kick;
	fCancel = cancel;
	
	// Create thread
	if(pthread_create(&bluetooth_thread, NULL, &bluetooth_read_loop, NULL)) {
		printf("Error creating thread\n");
		return 0;
	}
	return 1;
}

void destroy_bluetooth(){
	close(bt);
	pthread_cancel(bluetooth_thread);
}
