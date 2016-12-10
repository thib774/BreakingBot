#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_tacho.h"
#include "lever_pince.h"

/*Fait effectuer un tour de 90 degres au moteur controlant la pince pour la lever*/


void lever_pince(uint8_t sn, int max_speed)
{
        printf("levage de la pince...\n" );
       	set_tacho_speed_sp( sn, max_speed / 3 );
        set_tacho_ramp_up_sp( sn, 0 );
	set_tacho_ramp_down_sp( sn, 0 );
        set_tacho_position_sp( sn, 180 );
        set_tacho_command_inx( sn, TACHO_RUN_TO_REL_POS );

}
