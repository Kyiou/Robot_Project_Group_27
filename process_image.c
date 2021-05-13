#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>


#include <process_image.h>
#include <pi_regulator.h>

static  colors color;
static uint8_t color_ready;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//function to normalize the color values
uint16_t normalize(uint16_t rgb, uint16_t mean)
{
	if(rgb > mean)
	{
		rgb -= mean;
		return rgb;
	}
	return 0;
}

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

	//deactivate auto white balance for camera (1 activate)
	po8030_set_awb(0);

	//camera settings to adjust depending on ambient light
	//brightness from -128 to 127 (default 0)
	//contrast from 0 to 255 (default 64)
	po8030_set_brightness(0);
	po8030_set_contrast(64);

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

		uint16_t r = 0, g = 0, b = 0, mean = 0, distance =0;

		//get the distance to object
		 distance = VL53L0X_get_dist_mm();

		 //turn off the lights only if not at turning distance
		 if(distance > GOAL_DISTANCE)
		 {
			 clear_leds();
			 set_body_led(0);
		 }

		 color_ready = 0;

		 //analyze the color of a picture only if at a turning distance
		 if((distance <= GOAL_DISTANCE + ERROR_THRESHOLD && distance >= GOAL_DISTANCE - ERROR_THRESHOLD) && rotation_finished())
		 {
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

			r = normalize(r, mean);
			g = normalize(g, mean);
			b = normalize(b, mean);

			chprintf((BaseSequentialStream *)&SD3, "Color : r = %d , g = %d , b = %d, mean = %d \n", r, g, b, mean);

			//indicate the color based on experimental threshold white by default
			//color_ready is the signal that the color was analyzed
			if(mean < 3*ERROR_THRESHOLD)
			{
				set_body_led(1);
				color = BLACK;
			}
			else if((r > COLOR_THRESHOLD) && ((b < COLOR_THRESHOLD) && (g<COLOR_THRESHOLD)))
			{
				toggle_rgb_led(LED2, RED_LED, 1);
				toggle_rgb_led(LED8, RED_LED, 1);
				color = RED;
			}
			else if((g > COLOR_THRESHOLD) && ((b < COLOR_THRESHOLD) && (r<COLOR_THRESHOLD)))
			{
				toggle_rgb_led(LED2, GREEN_LED, 1);
				color = GREEN;
			}
			else if((b > COLOR_THRESHOLD) && ((r < COLOR_THRESHOLD) && (g<COLOR_THRESHOLD)))
			{
				toggle_rgb_led(LED8, BLUE_LED, 1);
				color =  BLUE;
			}
			else
			{
				color = WHITE;
			}
			color_ready = 1;
		 }
    }
}

colors get_color(void)
{
	//chprintf((BaseSequentialStream *)&SD3, "Color = %d \n", color);
	return color;
}

uint8_t ready(void)
{
	return color_ready;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
