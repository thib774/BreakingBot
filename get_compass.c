#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_sensor.h"
#include "get_compass.h"

float get_compass(uint8_t sn_compass) {

	float value_compass;
        if ( !get_sensor_value0(sn_compass, &value_compass )) {
               	value_compass = 0;
        }
        return(value_compass);
}

