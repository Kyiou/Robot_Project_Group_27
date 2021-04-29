#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <process_image.h>

static  colors color;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;

    while(1)
    {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint16_t r = 0, g = 0, b = 0, mean = 0;

	    for(uint16_t i = 0; i < (IMAGE_BUFFER_SIZE*2); i+=4)
	    {
	    	r += (uint16_t)img_buff_ptr[i/2]&0xF8;
	    	g += (uint16_t)(img_buff_ptr[i/2]&0x07)<<5 | (img_buff_ptr[i/2+1]&0xE0)>>3;
	    	b += (uint16_t)(img_buff_ptr[i/2+1]&0x1F)<<3;
	    }

	    r /= (IMAGE_BUFFER_SIZE/2);
	    g /= (IMAGE_BUFFER_SIZE/2);
	    b /= (IMAGE_BUFFER_SIZE/2);

	    mean = (r+g+b)/3;

	    clear_leds();
	    set_body_led(0);

		if(mean < 10)
		{
			set_body_led(1);
			color = BLACK;
		}
		else if(r > mean && r > 100)
		{
			toggle_rgb_led(LED2, RED_LED, 1);
			toggle_rgb_led(LED8, RED_LED, 1);
			color = RED;
		}
		else if(g > mean && g > 100)
		{
			toggle_rgb_led(LED2, GREEN_LED, 1);
			color = GREEN;
		}
		else if(b > mean && b > 100)
		{
			toggle_rgb_led(LED8, BLUE_LED, 1);
			color =  BLUE;
		}
		else
		{
			color = WHITE;
		}
    }
}

colors rgb_analyze(void)
{
	return color;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
