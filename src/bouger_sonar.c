#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "bouger_sonar.h"
#include "avancer_dst.h"
#include "reculer_dst.h"
#include "mes_sonar.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

float init_gyro;
int init_pos;
float value_sonar;

void bouger_sonar(uint8_t sn_right, uint8_t sn_left, uint8_t sn_gyro, uint8_t sn_sonar, uint8_t motor[2], float value, float gyro_val, float gyro_abs, FLAGS_T state, float dist)
{

		int max_speed;
		int speed;
		get_tacho_speed_sp(sn_right, &speed);
		get_tacho_max_speed(sn_right,&max_speed);
		mes_sonar(sn_sonar, value);
	if (abs(value_sonar - dist) > 5) {
        while (abs(value_sonar - dist) > 5) {
		if (abs(value_sonar - dist) < 20) {
			set_tacho_speed_sp(sn_right, max_speed / 7);
			set_tacho_speed_sp(sn_left, max_speed / 7);
		}
			if (value_sonar < dist) {
                reculer_dst(sn_right, sn_left, sn_gyro, motor, value, gyro_val, gyro_abs, state, dist-value_sonar);
			} else {
                avancer_dst(sn_right, sn_left, sn_gyro, motor, value, gyro_val, gyro_abs, state, value_sonar-dist);
			}
        mes_sonar(sn_sonar, value);
        }
	set_tacho_speed_sp(sn_right, speed);
	set_tacho_speed_sp(sn_left, speed);
	}
}
