#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "grab_droite.h"
#include "avancer_dst_sonar.h"
#include "mes_sonar_fast.h"
#include "poser_balle.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

float init_gyro;
float second_gyro;
int init_pos;
float value_sonar;

void grab_droite(uint8_t sn_right, uint8_t sn_left, uint8_t sn_clamp, uint8_t sn_gyro, uint8_t sn_sonar, uint8_t motor[2], float value, float gyro_val, float gyro_abs, FLAGS_T state)
{

    int max_speed;
    int speed;
    get_tacho_speed_sp(sn_right, &speed);
    get_tacho_max_speed(sn_right,&max_speed);
    set_tacho_speed_sp(sn_right, max_speed/7);
    set_tacho_speed_sp(sn_left, max_speed/7);

	second_gyro = init_gyro;
	float gyro_final = second_gyro + 60;
	float mini_sonar = 2500;
	float mini_angle = second_gyro;
	int pos1, pos2;
	get_tacho_position(sn_clamp, &pos1);
	//int i, iter;
	//iter =3;
	//for (i=0; i<iter; i++) {
	while(1) {
		mini_sonar = 2500;
		while (second_gyro < gyro_final) {
			set_tacho_position_sp( sn_right, -1 );
            set_tacho_position_sp( sn_left, 1 );
            multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
			do {
				get_tacho_state_flags( sn_right, &state );
            } while (state);
			while ( !get_sensor_value0(sn_gyro, &second_gyro )) {
			}
			mes_sonar_fast(sn_sonar, value);
			if (value_sonar < mini_sonar) {
				mini_sonar = value_sonar;
				mini_angle = second_gyro;
			}
			if (value_sonar < 170) {
				set_tacho_command_inx( sn_left, TACHO_STOP );
                                set_tacho_command_inx( sn_right, TACHO_STOP );
				set_tacho_speed_sp(sn_right, max_speed/20);
		                set_tacho_speed_sp(sn_left, max_speed/20);
				set_tacho_command_inx( sn_left, TACHO_RUN_FOREVER);
				set_tacho_command_inx( sn_right, TACHO_RUN_FOREVER);
				while (value_sonar <170){
                        	mes_sonar_fast(sn_sonar, value);
                                if (value_sonar < 50) {
                                	set_tacho_command_inx( sn_left, TACHO_STOP );
                                        set_tacho_command_inx( sn_right, TACHO_STOP );
                                        set_tacho_position_sp( sn_clamp, 480 );
                                        set_tacho_command_inx( sn_clamp, TACHO_RUN_TO_REL_POS);
                                        Sleep(500);
                                        break;
                                }
				}
                        }
			if (value_sonar < 170) {
                        	break;
                        }
		}

		printf("angle: %f\nval: %f\n",mini_angle,mini_sonar);

		second_gyro = mini_angle;
		if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
			if ( !get_sensor_value0(sn_gyro, &value )) {
				value = init_gyro;
			}
			gyro_val = value-second_gyro;
			gyro_abs = abs(gyro_val);
		}

		while (gyro_abs > 1.0) {
			set_tacho_command_inx( sn_left, TACHO_STOP );
			set_tacho_command_inx( sn_right, TACHO_STOP );
			if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
				if ( !get_sensor_value0(sn_gyro, &value )) {
					value = init_gyro;
				}
				gyro_val = value-second_gyro;
				gyro_abs = abs(gyro_val);
			}
			set_tacho_position_sp( sn_right, gyro_val-2 );
			set_tacho_position_sp( sn_left, -gyro_val+2 );
			multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
			//Sleep(gyro_abs*5);
			do {
				get_tacho_state_flags( sn_right, &state );
			} while (state);
			if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
                                if ( !get_sensor_value0(sn_gyro, &value )) {
                                        value = init_gyro;
                                }
                                gyro_val = value-second_gyro;
                                gyro_abs = abs(gyro_val);
                        }
		}

		set_tacho_speed_sp(sn_right, max_speed/20);
    		set_tacho_speed_sp(sn_left, max_speed/20);
		avancer_dst_sonar(sn_right, sn_left, sn_clamp, sn_gyro, sn_sonar, motor, value, gyro_val, gyro_abs, state, (float) mini_sonar * 2 / 5);
		set_tacho_speed_sp(sn_right, max_speed/7);
    		set_tacho_speed_sp(sn_left, max_speed/7);

		if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
			if ( !get_sensor_value0(sn_gyro, &value )) {
				value = init_gyro;
			}
			gyro_val = value-init_gyro;
			gyro_abs = abs(gyro_val);
		}

		while (gyro_abs > 1.0) {
			set_tacho_command_inx( sn_left, TACHO_STOP );
			set_tacho_command_inx( sn_right, TACHO_STOP );
			if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
				if ( !get_sensor_value0(sn_gyro, &value )) {
					value = init_gyro;
				}
				gyro_val = value-init_gyro;
				gyro_abs = abs(gyro_val);
			}
			set_tacho_position_sp( sn_right, gyro_val-2 );
			set_tacho_position_sp( sn_left, -gyro_val+2 );
			multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
			//Sleep(gyro_abs*5);
			do {
				get_tacho_state_flags( sn_right, &state );
			} while (state);
			if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){
                                if ( !get_sensor_value0(sn_gyro, &value )) {
                                        value = init_gyro;
                                }
                                gyro_val = value-init_gyro;
                                gyro_abs = abs(gyro_val);
                        }
		}
		get_tacho_position(sn_clamp, &pos2);
		if (pos2 != pos1) {
			break;
		}
	}
	// Remplacer 90 par 180 dans demi-tour
    set_tacho_speed_sp(sn_right, speed);
    set_tacho_speed_sp(sn_left, speed);
    //poser_balle(sn_clamp, state);
}
