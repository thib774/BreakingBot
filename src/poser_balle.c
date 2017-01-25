#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "poser_balle.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "ev3_port.h"


#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )


void poser_balle(uint8_t sn_clamp)
{
	set_tacho_position_sp( sn_clamp, -450 );
        set_tacho_command_inx( sn_clamp, TACHO_RUN_TO_REL_POS );
        Sleep(2000);
	//do {
		//get_tacho_state_flags( sn_clamp, &state );
	//} while (state);

}
