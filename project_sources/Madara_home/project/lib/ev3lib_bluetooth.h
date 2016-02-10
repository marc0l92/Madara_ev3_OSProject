#ifndef EV3LIB_BLUETOOTH_H
#define EV3LIB_BLUETOOTH_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

//#define SERV_ADDRESS "00:1E:0A:00:02:E1"     // Official
#define SERV_ADDRESS "00:1B:10:00:2A:EC"     // NetBook
//#define SERV_ADDRESS "30:3A:64:36:23:F0"     // Chiara
//#define SERV_ADDRESS "CC:52:AF:77:4C:3F"     // Team3

#define MY_ADDRESS   "00:17:E9:F5:AF:56"

#define BUFFER_SIZE 20

#define MSG_ACTION  0
#define MSG_ACK     1
#define MSG_LEAD    2
#define MSG_START   3
#define MSG_STOP    4
#define MSG_WAIT    5
#define MSG_CUSTOM  6
#define MSG_KICK	7
#define MSG_CANCEL	8

#define TEAM_ID     1

typedef void (*fAction_t)(uint8_t src, uint16_t angle, uint8_t dist, uint16_t speed);
typedef void (*fAck_t)(uint8_t src, uint16_t message_id, uint8_t state);
typedef void (*fLead_t)();
typedef void (*fStart_t)(uint8_t rank, uint8_t size, uint8_t p, uint8_t n);
typedef void (*fStop_t)();
typedef void (*fWait_t)(uint8_t src, uint8_t delay);
typedef void (*fKick_t)(uint8_t rank, uint8_t p, uint8_t n);
typedef void (*fCancel_t)(uint8_t dist);

void bluetooth_init();
void bluetooth_test_init(char *address, int port);

int send_action(uint8_t dst, uint16_t angle, uint8_t dist, uint16_t speed);
int send_ack(uint8_t dst, uint16_t msg_id, uint8_t state);
int send_lead(uint8_t dst);
int send_wait(uint8_t dst, uint16_t delay);
int send_cancel(uint8_t dst, uint8_t dist);
int bluetooth_register_and_start(fAction_t action, fAck_t ack, fLead_t lead, fStart_t start, fStop_t stop, fWait_t wait, fKick_t kick, fCancel_t cancel);

void destroy_bluetooth();

#endif
