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

static uint8_t rotation_done;

/***************************INTERNAL FUNCTIONS************************************/

/*
*@brief						simple PI regulator implementation
*
*@param		distance		how far from an object the robot is
*@param		goal			what is the distance desired to the object
*
*@retval 	speed			how fast the motors will go
*
*/
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

	//classical PI regulator
	speed = KP * error + KI * sum_error;

	//check for speed limit
    if (speed > MOTOR_MAX)
    {
        speed = MOTOR_MAX;
    }
    else if (speed < -MOTOR_MAX)
    {
        speed = -MOTOR_MAX;
    }

    return (int16_t)speed;
}

/*
*@brief						precise rotation of the robot
*
*@param		angle			the angle we want the robot to complete
*
*/
void rotate_robot (float angle)
{
	int32_t nb_step_to_turn;

	//transform the angle into a number of steps
	nb_step_to_turn = (int32_t)((angle * WHEEL_AXIS/WHEEL_PERIMETER) * NB_STEP);

	//sets a position before turning to count steps
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	//perform a rotation
	while ((abs(left_motor_get_pos()) < abs(nb_step_to_turn)) &&
			(abs(right_motor_get_pos()) < abs(nb_step_to_turn)))
	{
		if(angle < 0)
		{
			right_motor_set_speed(MOTOR_SPEED);
			left_motor_set_speed(-MOTOR_SPEED);
		}
		else
		{
			right_motor_set_speed(-MOTOR_SPEED);
			left_motor_set_speed(MOTOR_SPEED);
		}
	};

	//stops to improve the straight line move after
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}


/*
*@brief						set the rotation depending on the color
*
*@param		color			color obtained from analysis of the camera
*
*/
void rotation (colors color)
{
	float angle;

	//switch that sets the different actions depending on colors
	switch (color)
	{
		case BLACK: //stops if close to a dark body
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;

		case RED: //180 turn
			angle = -PI;
			rotate_robot (angle);
			break;

		case BLUE: //turns right
			angle = -PI/2;
			rotate_robot (angle);

			break;

		case GREEN: //turns left
			angle = PI/2;
			rotate_robot (angle);
			break;

		case WHITE: //stops if close to a luminous object
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			break;
	}
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t distance;

    while(TRUE)
    {
        time = chVTGetSystemTime();

        //get the distance from time of flight sensor
        distance = VL53L0X_get_dist_mm();

        rotation_done = TRUE;

        //stops the motor when around analyzing distance
        if(distance <= GOAL_DISTANCE + ERROR_THRESHOLD &&
           distance >= GOAL_DISTANCE - ERROR_THRESHOLD)
        {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);

        	//does the rotation only after it analyzed a color
        	if(ready())
        	{
				rotation_done = FALSE;
				rotation (get_color());
        	}
        }
        else
        {
        	//move straight to the next objective if the distance is greater than goal
    		right_motor_set_speed(pi_regulator(distance, GOAL_DISTANCE));
    		left_motor_set_speed(pi_regulator(distance, GOAL_DISTANCE));
        }
        //chprintf((BaseSequentialStream *)&SD3, "Distance = %d , PI_speed = %d \n", distance, pi_regulator(distance, GOAL_DISTANCE));
        //sets PI frequency to 100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(PI_PERIOD));
    }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/


uint8_t rotation_finished(void)
{
	return rotation_done;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
