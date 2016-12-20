#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "baisser_pince.h"
#include "ev3_tacho.h"



#define Sleep( msec ) usleep(( msec ) * 1000 )
void baisser_pince(uint8_t sn, int max_speed)
{
        printf("abaissement de la pince...\n" );
        set_tacho_speed_sp( sn, max_speed / 2 );
        set_tacho_ramp_up_sp( sn, 0 );
        set_tacho_ramp_down_sp( sn, 0 );
        set_tacho_position_sp( sn, -180 );
        set_tacho_command_inx( sn, TACHO_RUN_TO_REL_POS );

}


