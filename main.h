#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
//main
#define SEC						1000 //in [ms]

//process image
#define IMAGE_BUFFER_SIZE		640
#define COLOR_THRESHOLD			20 	//threshold for intensity of color
#define COLOR_MIN_MEAN			30
#define NB_COLOR				31
#define BRIGHTNESS				16  //brightness value
#define CONTRAST				32  //contrast value

//PI regulator
#define GOAL_DISTANCE 			100.0f	//[mm]
#define ERROR_THRESHOLD			10.0f	//[cm] because of the noise of the camera
#define KP						7.0f
#define KI 						0.3f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define WHEEL_DIAMETER			4.0f //cm	//must be measured on robot
#define PI						3.1415926536f
#define WHEEL_PERIMETER			(PI * WHEEL_DIAMETER) // cm
#define NB_STEP					1000
#define WHEEL_AXIS				2.55 //cm	//must be measured on robot
#define MOTOR_SPEED				250	//in steps during a turn for steady turns
#define MOTOR_MAX				500	//in steps low enough so the robot doesn't drift
#define PI_PERIOD				10	//ms	//corresponds to 100Hz

//Sound
#define START_THRESHOLD			3000 //in volume intensity
#define STOP_THRESHOLD			5*START_THRESHOLD //same as START_THRESHOLD

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
