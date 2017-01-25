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
#define SERV_ADDR   "98:01:A7:9F:09:00"     /* Whatever the address of the server is */
#define TEAM_ID     2                      /* Your team ID */

#define MSG_ACK     0
#define MSG_NEXT    1
#define MSG_START   2
#define MSG_STOP    3
#define MSG_CUSTOM  4
#define MSG_KICK    5
#define MSG_POSITION 6
#define MSG_BALL 	7

//Threads and mutex
//Robot
pthread_mutex_t mutex_moving=PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t condition_finisher= PTHREAD_COND_INITIALIZER;
pthread_cond_t condition_beginner= PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex_x_robot = PTHREAD_MUTEX_INITIALIZER; 
pthread_mutex_t mutex_y_robot = PTHREAD_MUTEX_INITIALIZER; 
int stop;
int moving; //moving =0 if beginner, 1 if finisher and 2 if waiting
int position; //1 for beginner right, 2 for beginner left, 3 for finisher right, 4 for finisher left


//Position global variables
//Robot
int16_t x_robot;
int16_t y_robot;
//Ball
int16_t x_ball;
int16_t y_ball;


int role; //0 for beginner, 1 for finisher

unsigned char rank = 0; //0 for beginner, 1 for finisher 
unsigned char side=0; //0 for right, 1 for left 
unsigned char length = 0;
unsigned char previous = 0xFF;
unsigned char next = 0xFF;

int s;

uint16_t msgId = 0;


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
uint8_t motor[2]={DESC_LIMIT,DESC_LIMIT};
float init_gyro;
float second_gyro;
float value_sonar;
int init_pos;
int case_pos = 0;
int sauver_pos;





int initialization(){
#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";

#endif
	if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
	//printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
	//printf( "Waiting tacho is plugged...\n" );

#endif
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	//printf( "*** ( EV3 ) Hello! ***\n" );

	//Check motors here
	if ( ev3_search_tacho_plugged_in(67,0, &sn1, 0 ) == 0) {
		printf("moteur droit debranche\n");
	}
	if ( ev3_search_tacho_plugged_in(68,0, &sn2, 0 ) == 0) {
		printf("moteur gauche debranche\n");
	}
	if ( ev3_search_tacho_plugged_in(65,0, &sn3, 0 ) == 0) {
		printf("moteur pince debranche\n");
	}

	//Preemptive motors stop
	set_tacho_command_inx( sn1, TACHO_STOP );
        set_tacho_command_inx( sn2, TACHO_STOP );
	set_tacho_command_inx( sn3, TACHO_STOP );

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
	if (ev3_search_sensor(LEGO_EV3_US, &sn_compass,0) == 0){
		printf("compass missing\n");
	}
	if (ev3_search_sensor(LEGO_EV3_US, &sn_color,0) == 0){
		printf("color missing\n");
	}
	if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
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



void* serverthread() /*thread sending the position every 2 seconds*/
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



void* messagethread(){ //always listen to the server
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


void* beginner(){
	char string[58];
	printf ("I'm the beginner...\n");
	
	if (position==1){ //Beginner of the right side

		//set the initial position of the robot
		pthread_mutex_lock(&mutex_x_robot);
		x_robot=30;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=25;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//the robot is moving so needs to send its position
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving);
		
		//From the right start to the middle to drop the ball
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 800);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 600);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 280);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 400);

				
		//drop the ball or drop it and send the message it was done
		poser_balle(sn3);

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=60;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=200;
		pthread_mutex_unlock(&mutex_y_robot);
				
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
	
	
		//From the middle where the ball was to the end 
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 200);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 1770);		

		//Preemptive motors stop
        	set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();
		
		//the robot has now to stop sending messages of its position		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=1;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
		
		
		//send the next message
		Sleep(3000);
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
		string[3] = next;
		string[4] = MSG_NEXT;
		write(s, string, 5);
		

		
	
	}
	else if (position==2){ //the robot is the left beginner 
		
		//set the initial position of the robot

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=-30;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=25;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//the robot is moving so needs to send its position
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving);
		
		//From the beginning to the zone where the ball has to be drop
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 800);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 600);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 280);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 400);
	
		//drop the ball	and send the message it was done	
		poser_balle(sn3);
			
		pthread_mutex_lock(&mutex_x_robot);
		x_robot=-60;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=200;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//message 
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
	 	string[3] = next;
	    	string[4] = MSG_BALL;
		string[5]=0x0; //drop the ball
	    	string[6] = x_robot;          // x 
		string[7] = 0x00;
	    	string[8] = y_robot;		//y 
		string[9]= 0x00;
		write(s, string, 10);
	
		//from the zone where the ball was to the end 
		reculer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 200);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 1770);
		

		//Preemptive motors stop
		set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();


		//the robot has now to stop sending messages of its position		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=1;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */

		Sleep(3000);
		
		//send the next message
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
		string[3] = next;
		string[4] = MSG_NEXT;
		write(s, string, 5);
	}

}	

void* finisher(){
	char string[58];
	printf ("I'm the finisher...\n");
	int temp=1;
	while (temp!=0)
	{
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		pthread_cond_wait(&condition_finisher,&mutex_moving);
		temp=0;
		printf("I'm no longer nwaiting\n");
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */	
		break;	
	}
	
	pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
	moving=0; //the robot is now moving
	pthread_mutex_unlock (&mutex_moving);
	

	if (position==3){ //Finisher of the right side

		//set the initial position of the robot

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=90;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=375;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//the robot is moving so needs to send its position
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving);
		
		//From the right end to the middle to grab the ball
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 800);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 600);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 460);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 80);
	
	

				
		//grab the ball or drop it and send the message it was done
		grab_droite(sn1, sn2, sn3, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state);

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=60;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=200;
		pthread_mutex_unlock(&mutex_y_robot);
				
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
	
	
		//From the middle where the ball was to the end 
				
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 280);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 1870);



		//Preemptive motors stop
        	set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();
		
		//the robot has now to stop sending messages of its position		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=1;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */
		
		
		//send the next message
		Sleep(3000);
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
		string[3] = next;
		string[4] = MSG_NEXT;
		write(s, string, 5);
		

		
	
	}
	else if (position==4){ //the robot is the left finisher 
		
		//set the initial position of the robot

		pthread_mutex_lock(&mutex_x_robot);
		x_robot=-90;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=375;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//the robot is moving so needs to send its position
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=0;
		pthread_mutex_unlock (&mutex_moving);
		
		//From the finish to the zone where the ball has to be grabbed				
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 800);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 600);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		bouger_sonar(sn1, sn2, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, 460);
		tourner_droite(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 80);	
	
		//grab the ball	and send the message it was done	
		
		grab_gauche(sn1, sn2, sn3, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state);
		
		pthread_mutex_lock(&mutex_x_robot);
		x_robot=-60;
		pthread_mutex_unlock(&mutex_x_robot);

		pthread_mutex_lock(&mutex_y_robot);
		y_robot=200;
		pthread_mutex_unlock(&mutex_y_robot);
		
		//message 
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
	
		//from the zone where the ball was to the end 
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 280);
		tourner_gauche(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state);
		avancer_dst(sn1, sn2, sn_gyro, motor, value, gyro_val, gyro_abs, state, 1870);
		

		//Preemptive motors stop
		set_tacho_command_inx( sn1, TACHO_STOP );
        	set_tacho_command_inx( sn2, TACHO_STOP );
		ev3_uninit();


		//the robot has now to stop sending messages of its position		
		pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
		moving=1;
		pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */

		Sleep(3000);
		
		//send the next message
		*((uint16_t *) string) = msgId++;
		string[2] = TEAM_ID;
		string[3] = next;
		string[4] = MSG_NEXT;
		write(s, string, 5);
	}

	else{
	printf("Nothing to be done here\n");	
	}




		
}


int main() {
	struct sockaddr_rc addr = { 0 };
	int status;
	
	pthread_t myBeginnerThread;
	pthread_t myFinisherThread;
	pthread_t myMessageThread;
	pthread_t myServerThread;
	
	initialization();

	gyro_val = 90;
	gyro_abs = 90;
	
	stop=0;
	
	//attraper_balle_init(sn3, sn_sonar, value);
	/* allocate a socket */
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	/* set the connection parameters (who to connect to) */
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba (SERV_ADDR, &addr.rc_bdaddr);

	/* connect to server */
	status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

	/* if connected */
	if( status == 0 ) {
		char string[58];

		/* Wait for START message */
		read_from_server (s, string, 9);
		if (string[4] == MSG_START) {
			printf ("Received start message!\n");
			rank = (unsigned char) string[5];
			side= (unsigned char) string[6];
			next = (unsigned char) string[7];
		}
		

		if (rank==0){
			attraper_balle_init(sn3, sn_sonar, value);
		}
		pthread_create(&myMessageThread,NULL,messagethread,NULL);
		pthread_create(&myServerThread,NULL,serverthread,NULL);		

		if (rank == 0){
			if (side ==0){
				position=1; //right beginner
			}
			else if (side == 1){
				position =2; //left beginner
			}		
			pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
			moving=0;
			pthread_cond_signal(&condition_beginner);
			pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */ 
		}
		else{
			if (side == 0){
				position=3; //right finisher
			}		
			else if (side == 1){
				position=4; //left finisher
			}	
					
			pthread_mutex_lock(&mutex_moving); /* Lock the mutex on moving */
			moving=1;
			pthread_cond_signal(&condition_finisher);
			pthread_mutex_unlock (&mutex_moving); /* Unlocking the mutex */			
		} 		
		
		if (position == 1 || position ==2){ //beginner
			pthread_create(&myBeginnerThread,NULL,beginner,NULL);
			pthread_join(myBeginnerThread,NULL);
			}
		else if (position==3 || position == 4){ //finisher
			pthread_create(&myFinisherThread,NULL,finisher,NULL);
			pthread_join(myFinisherThread,NULL);
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
	pthread_cancel(myServerThread);
	
	//close the connection with the server
	close(s);
	return 0;
}


