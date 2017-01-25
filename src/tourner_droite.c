#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "tourner_droite.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

float init_gyro;

void tourner_droite(uint8_t sn_right, uint8_t sn_left, uint8_t sn_gyro, uint8_t motor[2], float value, float gyro_val, float gyro_abs, FLAGS_T state)
{
        //printf("Tourne droite...\n" );
        init_gyro = init_gyro + 90;

        if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){

                if ( !get_sensor_value0(sn_gyro, &value )) {

                        value = init_gyro + 90;

                }

                gyro_val = value-init_gyro;
                gyro_abs = abs(gyro_val);

        }

	while (gyro_abs > 2.0) {

		//motors stop
		set_tacho_command_inx( sn_left, TACHO_STOP );

		set_tacho_command_inx( sn_right, TACHO_STOP );
		if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)){

			if ( !get_sensor_value0(sn_gyro, &value )) {

				value = init_gyro + 90;

			}

			gyro_val = value-init_gyro;
			gyro_abs = abs(gyro_val);

			//printf( "\r(%f) \n", gyro_val);
			//fflush( stdout );

		}
                        set_tacho_position_sp( sn_right, gyro_val );
			set_tacho_position_sp( sn_left, -gyro_val );
			multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
			//Sleep(gyro_abs*5);
			do {
				get_tacho_state_flags( sn_right, &state );
			} while (state);

	}

}
