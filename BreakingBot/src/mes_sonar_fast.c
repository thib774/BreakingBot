#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "mes_sonar_fast.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

float value_sonar;

void mes_sonar_fast(uint8_t sn_sonar, float value)
{
    //printf("Mesure Sonar...\n" );
        int i;
        int number = 10;
        float mean = 0.0;
        value = 0;
        for (i=0; i<number; i++) {
        if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
                if ( get_sensor_value0(sn_sonar, &value )) {
                        mean = mean + value;
                }
        }
        Sleep(5);
        }
        value_sonar = mean / number;
}
