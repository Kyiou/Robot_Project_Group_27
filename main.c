#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <spi_comm.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <audio/play_sound_file.h>


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg =
	{
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();

    //start the USB communication
    usb_start();

    //starts the camera
    dcmi_start();
	po8030_start();

	//starts distance measurement
	VL53L0X_start();

	//init motors
	motors_init();

	//starts communication
	spi_comm_start();

	//starts the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

	//starts microphone
	mic_start(NULL);
    dac_start();
    playMelodyStart();

    /* Infinite loop. */
    while (TRUE)
    {
    	//waits 1 second
        chThdSleepMilliseconds(SEC);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

