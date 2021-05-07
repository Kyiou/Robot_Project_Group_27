#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define WHEEL_DIAMETER			5.35f // cm // valeur à mesurer
#define PI						3.1415926536f
#define WHEEL_PERIMETER			(PI * WHEEL_DIAMETER) // cm
#define NB_STEP					1000
#define WHEEL_AXIS				5 // cm // valeur à mesurer

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal)
{
	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error ;//+ KI * sum_error;

	chprintf((BaseSequentialStream *)&SD3, "Distance = %d\n",VL53L0X_get_dist_mm());

	if(VL53L0X_get_dist_mm() > 100)
	{
		speed = fabs(speed);
	}

    return (int16_t)speed;
}

void rotate_robot (float angle)
{
	int32_t nb_step_to_turn;

	nb_step_to_turn = (int32_t)((angle * WHEEL_AXIS/WHEEL_PERIMETER) * NB_STEP);

	left_motor_set_pos(0);
	right_motor_set_pos(0);
	while ((abs(left_motor_get_pos()) < abs(nb_step_to_turn)) && (abs(right_motor_get_pos()) < abs(nb_step_to_turn)))
	{
		right_motor_set_speed(-250);
		left_motor_set_speed(250);
	};
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void rotation (colors color)
{
	float angle;

	switch (color)
	{
		case BLACK: //turns opposite side
			angle = -PI;
			rotate_robot (angle);
			break;

		case RED: //stops
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;

		case BLUE: //turns right
			angle = -PI/2;
			rotate_robot (angle);
			break;

		case GREEN: //turns left
			angle = PI/2;
			rotate_robot (angle);
			break;

		case WHITE: //does nothing
			break;
	}
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        if(VL53L0X_get_dist_mm() <=100)
        {
        	//rotation (get_color());

        }
        else
        {
    		right_motor_set_speed(pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE));
    		left_motor_set_speed(pi_regulator(VL53L0X_get_dist_mm(), GOAL_DISTANCE));

        }

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
