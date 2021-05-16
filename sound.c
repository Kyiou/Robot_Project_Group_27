#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <audio/play_melody.h>
#include <audio/microphone.h>
#include <main.h>
#include <leds.h>


uint8_t start_stop(void)
{
	static uint8_t start = TRUE;
	static uint8_t signal = FALSE;

	uint16_t volume;

	//average the volume of the 4 mics
	volume = (mic_get_volume(MIC_FRONT)+mic_get_volume(MIC_RIGHT)+
			  mic_get_volume(MIC_BACK)+mic_get_volume(MIC_LEFT))/4;

	//check for starting signal, put thread to sleep so no double signal, led indication
	if(volume > START_THRESHOLD && start)
	{
		set_front_led(TRUE);
		signal = TRUE;
		start = FALSE;
		chThdSleepMilliseconds(SEC);
		set_front_led(FALSE);
		return signal;
	}
	//check for stop and plays a melody to indicate the stop
	else if(volume > STOP_THRESHOLD && !start)
	{
		signal = FALSE;
		start = TRUE;
		playMelody(MARIO_FLAG, ML_SIMPLE_PLAY, NULL);
		chThdSleepMilliseconds(SEC);
		return signal;
	}
	return signal;
}


