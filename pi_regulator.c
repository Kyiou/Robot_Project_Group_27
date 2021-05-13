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

#define WHEEL_DIAMETER			4.0f // cm // valeur à mesurer
#define PI						3.1415926536f
#define WHEEL_PERIMETER			(PI * WHEEL_DIAMETER) // cm
#define NB_STEP					1000
#define WHEEL_AXIS				2.55 // cm // valeur à mesurer
#define MOTOR_SPEED				250	 //in steps during a turn for steady turns
#define MOTOR_MAX				500	 //in steps low enough so the robot doesn't drift

static uint8_t rotation_done;

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

    //prevents the robot from going back when too far of the object
	if(distance > GOAL_DISTANCE)
	{
		speed = fabs(speed);
	}

    return (int16_t)speed;
}

void rotate_robot (float angle)
{
	int32_t nb_step_to_turn;

	nb_step_to_turn = (int32_t)((angle * WHEEL_AXIS/WHEEL_PERIMETER) * NB_STEP);

	//stops to improve the rotation after
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	//perform a rotation
	while ((abs(left_motor_get_pos()) < abs(nb_step_to_turn)) && (abs(right_motor_get_pos()) < abs(nb_step_to_turn)))
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

    while(1)
    {
        time = chVTGetSystemTime();

        distance = VL53L0X_get_dist_mm();

        rotation_done = 1;

        //stops the motor when around analyzing distance
        if(distance <= GOAL_DISTANCE + ERROR_THRESHOLD && distance >= GOAL_DISTANCE - ERROR_THRESHOLD)
        {
        	right_motor_set_speed(0);
        	left_motor_set_speed(0);

        	if(ready())
        	{
        		rotation_done = 0;
        		rotation (get_color());
        	}
        }
        else
        {
    		right_motor_set_speed(pi_regulator(distance, GOAL_DISTANCE));
    		left_motor_set_speed(pi_regulator(distance, GOAL_DISTANCE));

        }

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

uint8_t rotation_finished(void)
{
	return rotation_done;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
