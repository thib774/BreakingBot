#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "attraper_balle_init.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )


void attraper_balle_init(uint8_t sn_clamp, uint8_t sn_sonar, float value)
{
	while(1){
        //get the value of the sonar
        if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
            if ( !get_sensor_value0(sn_sonar, &value )) {
                value = 100;
            }
        }
            if (value < 50) {
                set_tacho_position_sp( sn_clamp, 480 );
		set_tacho_command_inx( sn_clamp, TACHO_RUN_TO_REL_POS);
		Sleep(500);
		break;
            }
    }
       

}
