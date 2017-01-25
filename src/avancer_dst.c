#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "avancer_dst.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

float init_gyro;
int init_pos;

void avancer_dst(uint8_t sn_right, uint8_t sn_left, uint8_t sn_gyro, uint8_t motor[2], float value, float gyro_val, float gyro_abs, FLAGS_T state, float dist)
{

		int max_speed;
		int pos, save_pos;
		get_tacho_position(sn_right, &pos);
		dist = (int) dist * 360 / 200;
		int final_pos = pos + dist;
        multi_set_tacho_command_inx(motor,TACHO_RUN_FOREVER);
		while(pos<final_pos){

            //get the value of the sonar

                if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){

                        if ( !get_sensor_value0(sn_gyro, &value )) {
                                value = init_gyro;
                        }

                        gyro_val = value-init_gyro;
                        gyro_abs = abs(gyro_val);

                        //printf( "\r(%f) \n", gyro_val);
                        //fflush( stdout );

                }



                //break if an obstacle is to close
				if (gyro_abs > 2.0) {
					get_tacho_position(sn_right, &save_pos);
					while (gyro_abs > 2.0) {

                        //motors stop
                        set_tacho_command_inx( sn_left, TACHO_STOP );
                        set_tacho_command_inx( sn_right, TACHO_STOP );
                        if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){

                                if ( !get_sensor_value0(sn_gyro, &value )) {

                                        value = init_gyro;

                                }

                                gyro_val = value-init_gyro;
                                gyro_abs = abs(gyro_val);

                                //printf( "\r(%f) \n", gyro_val);
                                //fflush( stdout );

                        }
			get_tacho_max_speed(sn_right,&max_speed);
			set_tacho_speed_sp( sn_right, max_speed / 10 );
                        set_tacho_speed_sp( sn_left, max_speed / 10 );
                        set_tacho_position_sp( sn_right, gyro_val-2 );
                        set_tacho_position_sp( sn_left, -gyro_val+2 );
                        multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
                        //Sleep(gyro_abs*5);
						do {
							get_tacho_state_flags( sn_right, &state );
						} while (state);
					}
					get_tacho_position(sn_right, &pos);
					save_pos = pos - save_pos;
					final_pos = final_pos + save_pos;
                        		set_tacho_speed_sp( sn_right, max_speed / 3 );
                        		set_tacho_speed_sp( sn_left, max_speed / 3);
					multi_set_tacho_command_inx(motor,TACHO_RUN_FOREVER);
				}


                get_tacho_position(sn_right, &pos);

            }

			set_tacho_command_inx( sn_left, TACHO_STOP );
            set_tacho_command_inx( sn_right, TACHO_STOP );

}

