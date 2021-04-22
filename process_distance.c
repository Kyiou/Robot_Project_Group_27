/*
 * process_distance.c
 *
 *  Created on: 22 avr. 2021
 *      Author: camil
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <sensors/VL53L0X/VL53L0X.h>

#include "process_distance.h"

void get_TOF_distance(void)
{
	chprintf((BaseSequentialStream *)&SDU1, "DISTANCE SENSOR\r\n");
	chprintf((BaseSequentialStream *)&SDU1, "%d\r\n\n", VL53L0X_get_dist_mm());
}

static THD_WORKING_AREA(waProcessDistance, 1024);
static THD_FUNCTION(ProcessDistance, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    while(1)
    {
    	time = chVTGetSystemTime();
    	get_TOF_distance();
    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}
