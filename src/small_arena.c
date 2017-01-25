#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <pthread.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "tourner_droite.h"
#include "tourner_gauche.h"
#include "avancer_dst.h"
#include "reculer_dst.h"
#include "demi_tour.h"
#include "mes_sonar.h"
#include "bouger_sonar.h"
#include "attraper_balle_init.h"
#include "poser_balle.h"
#include "grab_gauche.h"
#include "grab_droite.h"


#define Sleep( msec ) usleep(( msec ) * 1000 )     
#define SERV_ADDR "7C:5C:F8:45:D4:DA"  /* Whatever the address of the server is */

#define TEAM_ID     2                      /* Your team ID */

#define MSG_ACK     0
#define MSG_NEXT    1
#define MSG_START   2
#define MSG_STOP    3
#define MSG_CUSTOM  4
#define MSG_KICK    5
#define MSG_POSITION 6
#define MSG_BALL 	7

/*Threads and mutex*/
//Robot
pthread_mutex_t mutex_moving=PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condition_finisher= PTHREAD_COND_INITIALIZER;
pthread_cond_t condition_beginner= PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex_x_robot = PTHREAD_MUTEX_INITIALIZER; 
pthread_mutex_t mutex_y_robot = PTHREAD_MUTEX_INITIALIZER; 
void* positionhread();
//Server
void* serverthread();
void* messagethread();

/* If begginer, the robot should do 1->2, then send a next message. Then, wait for another next message and do 2->1 and 1->2 without sending a next message when in 1 */

/*If finisher, the robot should do 3->4, then 4->3 and then send a next message*/


//Position global variables
//Robot
int16_t x_robot;
int16_t y_robot;
int stop;
int moving; //boolean :moving = 0 if the robot is moving and 1 if not
int position; //1 for bottom right, 2 for top right, 3 for top left, 4 for bottom left
//Ball
int16_t x_ball;
int16_t y_ball;

unsigned char rank = 0; //0 for beginner, 1 for finisher 
unsigned char length = 0;
unsigned char previous = 0xFF;
unsigned char next = 0xFF;

int s;

uint16_t msgId = 0;

//All the global variables needed for the robot to move
uint8_t sn1;
uint8_t sn2;
uint8_t sn3;
FLAGS_T state = 1;
uint8_t sn_color;
uint8_t sn_sonar;
uint8_t sn_gyro;
uint8_t sn_compass;
int max_speed;
float value = 0;
float gyro_val;
float gyro_abs;
float init_gyro;
float second_gyro;
float value_sonar;
int init_pos;
int case_pos = 0;
int sauver_pos;
uint8_t motor[2]={DESC_LIMIT,DESC_LIMIT};



/*Init all the sensors, the motors, and tell if they are plugged or not */

int initialization(){
#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";

#endif
	if ( ev3_init() == -1 ) return ( 1 );

	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	//printf( "*** ( EV3 ) Hello! ***\n" );

	//Check motors here
	if ( ev3_search_tacho_plugged_in(67,0, &sn1, 0 ) == 0) {
		printf("Please plugg right engine\n");
	}
	if ( ev3_search_tacho_plugged_in(68,0, &sn2, 0 ) == 0) {
		printf("Please plugg left engine\n");
	}
	if ( ev3_search_tacho_plugged_in(65,0, &sn3, 0 ) == 0) {
		printf("Please plugg pliers' motor\n");
	}

	//Preemptive motors stop
	set_tacho_command_inx( sn1, TACHO_STOP );
        set_tacho_command_inx( sn2, TACHO_STOP );
	set_tacho_command_inx( sn3, TACHO_STOP );

	
	//Set the speed of the motors and their reference position
	if ( ev3_search_tacho_plugged_in(67,0, &sn1, 0 )) {
		get_tacho_max_speed(sn1,&max_speed);
		set_tacho_stop_action_inx( sn1, TACHO_COAST );
		set_tacho_speed_sp( sn1, max_speed / 4);
		set_tacho_time_sp( sn1, 1000 );
		set_tacho_ramp_up_sp( sn1, 0 );
		set_tacho_ramp_down_sp( sn1, 0 );
	}

	if ( ev3_search_tacho_plugged_in(68,0, &sn2, 0 )) {
		get_tacho_max_speed(sn2, &max_speed);
		set_tacho_stop_action_inx( sn2, TACHO_COAST );
		set_tacho_speed_sp( sn2, max_speed / 4);
                set_tacho_time_sp( sn2, 1000 );
                set_tacho_ramp_up_sp( sn2, 0 );
                set_tacho_ramp_down_sp( sn2, 0 );
	}

	if ( ev3_search_tacho_plugged_in(65,0, &sn3, 0 )) {
                get_tacho_max_speed(sn3, &max_speed);
                set_tacho_stop_action_inx( sn3, TACHO_COAST );
                set_tacho_speed_sp( sn3, max_speed / 3);
                set_tacho_time_sp( sn3, 1000 );
                set_tacho_ramp_up_sp( sn3, 0 );
                set_tacho_ramp_down_sp( sn3, 0 );
		set_tacho_position_sp( sn3, 0 );
        }

	ev3_sensor_init();
	
	//Check sensors here
	if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0) == 0){
		printf("SONAR missing\n");
	}
	if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)==0){
		printf("GYRO missing\n");		
		while ( !get_sensor_value0(sn_gyro, &init_gyro )) {
		}
	}

	get_tacho_position(sn1, &init_pos);

	set_tacho_command_inx( sn1, TACHO_STOP );
        set_tacho_command_inx( sn2, TACHO_STOP );
        set_tacho_command_inx( sn3, TACHO_STOP );


	motor[0]=sn1;
	motor[1]=sn2;

	Sleep(400);
	return 0;

}




void debug (const char *fmt, ...) {
	va_list argp;

	va_start (argp, fmt);

	vprintf (fmt, argp);

	va_end (argp);
}

// Function to read the data of the server 
int read_from_server (int sock, char *buffer, size_t maxSize) {
	int bytes_read = read (sock, buffer, maxSize);

	if (bytes_read <= 0) {
		fprintf (stderr, "Server unexpectedly closed connection...\n");
		close (s);
		exit (EXIT_FAILURE);
	}

	printf ("[DEBUG] received %d bytes\n", bytes_read);

	return bytes_read;
}


//Thread sending the position every 2 seconds
void* serverthread() 
{
	
	char string[58];	
	while(1){
		if (moving == 0){
			Sleep(2000);
			pthread_mutex_lock(&mutex_x_robot); /*Lock the mutex on x and y*/
			pthread_mutex_lock(&mutex_y_robot);
			/*Create the message to be send to the server */		
			*((uint16_t *) string) = msgId++;
			string[2] = TEAM_ID;
			string[3] = 0xFF;
			string[4] = MSG_POSITION;
			string[5] = x_robot;          /* x */
			string[6] = 0x00;
			string[7] = y_robot;		/* y */
			string[8]= 0x00;
			write(s, string, 9); /*Send the message to the server*/
			
			pthread_mutex_unlock(&mutex_x_robot);/*Unlock the mutex on x and y*/
			pthread_mutex_unlock(&mutex_y_robot);		
		}
		
	}
	pthread_exit(NULL); /* End of the thread */
}


//Thread calculating the position of the robot in real time
void* positionthread()	
{
	while(1){
		if (case_pos == 0) {
			mes_sonar(sn_sonar, value);
			pthread_mutex_lock(&mutex_y_robot);
			y_robot = (int16_t) (value_sonar / 10);
			pthread_mutex_unlock(&mutex_y_robot);
		} 
		else if (case_pos == 1) {
			mes_sonar(sn_sonar, value);
			pthread_mutex_lock(&mutex_y_robot);			
			y_robot = (int16_t) (20 - (value_sonar / 10));
			pthread_mutex_unlock(&mutex_y_robot);
		} 
		else if (case_pos == 2) {
			mes_sonar(sn_sonar, value);
			pthread_mutex_lock(&mutex_x_robot);
			x_robot = (int16_t) (value_sonar / 10);
			pthread_mutex_unlock(&mutex_x_robot);
		} 
		else if (case_pos == 3) {
			mes_sonar(sn_sonar, value);
			pthread_mutex_lock(&mutex_x_robot);
			x_robot = (int16_t) (12 - (value_sonar / 10));
			pthread_mutex_unlock(&mutex_x_robot);
		}
		else if (case_pos == 4) {

			int pos, old;
			old = sauver_pos;
			get_tacho_position(sn1, &sauver_pos);
			pos = old - sauver_pos;
			pthread_mutex_lock(&mutex_x_robot);
			x_robot = (int16_t) (x_robot - (pos * 180 / 360));
			pthread_mutex_unlock(&mutex_x_robot);
		} 
		else if (case_pos ==5) {
			int pos, old;
			old = sauver_pos;
			get_tacho_position(sn1, &sauver_pos);
			pos = old - sauver_pos;
			pthread_mutex_lock(&mutex_x_robot);
			x_robot = (int16_t) (x_robot + (pos * 180 / 360));
			pthread_mutex_unlock(&mutex_x_robot);
		 }
	}
}


//Thread listening to the server
void* messagethread(){ 
	char received[58];
	while(1){
		read_from_server (s, received, 58);
		switch(received[4]){
			case MSG_STOP:
				stop=1;
				printf("Stop\n");
				continue;
			case MSG_BALL:
				//I know where the ball is
				x_ball=received[7]*100+received[6];
				y_ball=received[9]*100+received[8];				
				printf("x_ball = %d\n",x_ball);
				printf("y_ball = %d\n",y_ball);  
				continue;
			case MSG_KICK:
				stop=1;
				continue;
			case MSG_NEXT:
				pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
				moving=0;
				if (position == 1 || position==2){pthread_cond_signal(&condition_beginner);}				
				else {pthread_cond_signal(&condition_finisher);}
				pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
				printf("next message received\n");				
			default:
				printf ("Ignoring message %d\n", received[4]);				
		}

	}
	pthread_exit(NULL); /* End of the thread */

}

//Thread of the beginner
void* beginner(){
	char string[58];
	printf ("I'm the beginner...\n");
	
	
	//When the beginner is in position 2, it has to wait for the NEXT message of the finisher
	int temp=1;
	if (position==2){
		while (temp!=0){
			pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
			pthread_cond_wait(&condition_beginner,&mutex_moving);
			moving=0;
			printf("I'm no longer waiting\n");
			pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */	
		break;	
		}
	}
	
	if (position==1){ 
		//The beginner is in position 1 and has to go to position 2, and then send a next message
		
		//set the initial position of the robot

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=90;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=20;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//the robot is moving so needs to send its position
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving);
		
		//From 1 to the middle to drop the ball
		
		case_pos = 0;			
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 930);
		case_pos = 6;
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		case_pos = 3;
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 436);
		case_pos = 6;
		demi_tour(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 10);
			
		//drop the ball or drop it and send the message it was done
		poser_balle(sn3);
				
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
	 	string[3] = next;
	    	string[4] = MSG_BALL;
		string[5]=0x0;
	    	string[6] = x_robot;          // x 
		string[7] = 0x00;
	    	string[8] = y_robot;		//y 
		string[9]= 0x00;
		write(s, string, 10);
		
		//From the middle where the ball was to the position 2 (top right) 
		case_pos = 4;
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 250);
		case_pos = 6;		
		demi_tour(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		case_pos = 3;		
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 160);
		case_pos = 6;		
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		case_pos = 1;		
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 230);

		//Preemptive motors stop
        	set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();
		
		//the robot has now to stop sending messages of its position		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=1;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
		
		//robot is now in position 2
		position=2;
		
		//send the next message
		Sleep(3000);
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
		string[3] = next;
		string[4] = MSG_NEXT;
		write(s, string, 5);
	}

	else if (position==2){ //the robot has to go from 2 to 1, and then from 1 to 2 without sending a message
		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
		
		//From 2 to the zone where the ball is
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 750);
		case_pos = 6;		
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		case_pos = 3;		
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 510);
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 300);
		case_pos = 6;		
	
		//search for the ball		
		grab_droite(sn1, sn2, sn3, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state);
			
		//pick the ball and send the message it was done
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
	 	string[3] = next;
	    	string[4] = MSG_BALL;
		string[5]=0x1; //pick the ball
	    	string[6] = x_robot;          // x 
		string[7] = 0x00;
	    	string[8] = y_robot;		//y 
		string[9]= 0x00;
		write(s, string, 10);
	
		//from the zone where the ball was to position 1
		case_pos = 3;		
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 230);
		case_pos = 6;		
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		case_pos = 0;	
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 230);
		
		//Preemptive motors stop
        	set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();
	
		//we are now in position 1 and need to go back to 2
		position=1;
	}

}	


//Thread of the finisher
void* finisher(){
	char string[58];
	printf ("I'm the finisher...\n");
	int temp=1;

	//Waiting for the NEXT message of the beginner (wait for the condition on moving delivered by the messagethread)
	while (temp!=0)
	{
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		pthread_cond_wait(&condition_finisher,&mutex_moving);
		temp=0;
		moving=0;
		printf("I'm no longer nwaiting\n");
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */	
		break;	
	}
	
	//The  robot is moving from the start position to the zone where the ball is 
	case_pos = 1;
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 700);
	case_pos = 6;		
	tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	case_pos = 2;		
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 810);
	case_pos = 6;		
	
	//The robot has now to grab the ball
	printf("I try to grab");
	grab_gauche(sn1, sn2, sn3, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state);	
	

	//Pick the ball and send a BALL message it was done	
	*((uint16_t *) string) = msgId++;
	string[2] = TEAM_ID;
	string[3] = next;
    	string[4] = MSG_BALL;
	string[5]=0x1;
    	string[6] = x_robot;          // x 
	string[7] = 0x00;
    	string[8] = y_robot;		//y 
	string[9]= 0x00;
	write(s, string, 10);

	printf("I have grabbed");
	

	case_pos = 2;		
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 436);
	case_pos = 6;		
	tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	case_pos = 0;	
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 230);

	position=4;	


	//from 4 to the middle to drop the ball
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 930);
	case_pos = 6;
	tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	case_pos = 2;
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 436);
	case_pos = 6;
	demi_tour(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 10);
	
	


	poser_balle(sn3);			
	//drop the ball or drop it and send the message it was done
	*((uint16_t *) string) = msgId++;
	string[2] = TEAM_ID;
 	string[3] = next;
    	string[4] = MSG_BALL;
	string[5]=0x0;
    	string[6] = x_robot;          // x 
	string[7] = 0x00;
    	string[8] = y_robot;		//y 
	string[9]= 0x00;
	write(s, string, 10);
	
	
	//From the middle where the ball was dropped to the start zone
	case_pos = 5;
	reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 250);
	case_pos = 6;		
	demi_tour(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	case_pos = 2;		
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 160);
	case_pos = 6;		
	tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
	case_pos = 1;		
	bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 230);
		
		
	//Preemptive motors stop
	set_tacho_command_inx( sn1, TACHO_STOP );
	set_tacho_command_inx( sn2, TACHO_STOP );
	//set_tacho_command_inx( sn3, TACHO_STOP );
	
	ev3_uninit();

	

	//the robot is not moving anymore: don't need to send a position message 
	pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
	moving=1; //robot is stopped
	pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
	
	Sleep(3000);

	//send the next message
	*((uint16_t *) string) = msgId++;
	string[2] = TEAM_ID;
	string[3] = next;
	string[4] = MSG_NEXT;
	write(s, string, 5);

	position=3; // robot is now in position 3

		
}


int main() {
	struct sockaddr_rc addr = { 0 };
	int status;
	
	pthread_t myBeginnerThread;
	pthread_t myFinisherThread;
	pthread_t myMessageThread;
	pthread_t myPositionThread;
	pthread_t myServerThread;
	
	initialization();

	gyro_val = 90;
	gyro_abs = 90;
	
	stop=0;
	
	//allocate a socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to) 
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba (SERV_ADDR, &addr.rc_bdaddr);

	// connect to server
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

	// if connected
	if( status == 0 ) {
		char string[58];

		//Wait for START message 
		read_from_server (s, string, 9);
		if (string[4] == MSG_START) {
			printf ("Received start message!\n");
			rank = (unsigned char) string[5];
			next = (unsigned char) string[7];
		}
		
		// Creation of the threads to receive the messages from the server, modify and send the position to the server		
		pthread_create(&myMessageThread,NULL,messagethread,NULL);
		pthread_create(&myPositionThread,NULL,positionthread,NULL);
		pthread_create(&myServerThread,NULL,serverthread,NULL);		
		
		//If we are the beginner, we need to provide the robot a ball for him to grab it 
		if (rank == 0){
			attraper_balle_init(sn3, sn_sonar, value);
		}
				
		if (rank == 0){ //we are the beginner 
			position=1;			
			pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
			moving=0;
			pthread_cond_signal(&condition_beginner);
			pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */ 
		}
		else{ //we are the finisher 
			pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
			moving=1;
			position=3;
			pthread_cond_signal(&condition_finisher);
			pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */			
		} 		
		
		//start the  infinite loop of movements 
		while (stop !=1){
			if (position == 1 || position ==2){ //beginner
				pthread_create(&myBeginnerThread,NULL,beginner,NULL);
				pthread_join(myBeginnerThread,NULL);
				continue;
			}
			else if (position==3 || position == 4){ //finisher
				pthread_create(&myFinisherThread,NULL,finisher,NULL);
				pthread_join(myFinisherThread,NULL);
				continue;
			}
					
		}
		close (s);
	
		sleep (5);

	} 
	else {
		fprintf (stderr, "Failed to connect to server...\n");
		sleep (2);
		exit (EXIT_FAILURE);
	}

	//delete all the threads
	pthread_cancel(myMessageThread);
	pthread_cancel(myPositionThread);
	pthread_cancel(myServerThread);
	
	//close the connection with the server
	close(s);
	return 0;
}


