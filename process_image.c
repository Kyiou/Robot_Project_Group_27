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

/***************************INTERNAL FUNCTIONS************************************/

/*
*@brief					function to normalize the color value
*						by subtracting the average due to
*						ambient light
*
*@param	rgb				intensity of one color channel
*@param	mean			average of the three color channel
*
*/
uint16_t normalize(uint16_t rgb, uint16_t mean)
{
	if(rgb > mean)
	{
		rgb -= mean;
		return rgb;
	}
	return 0;
}

/*
*@brief						extract the color from the picture
*
*@param	rgb[]				table with colors red, green, blue
*@param	*img_buff_ptr		pointer to the array filled with the last image
*
*/
void analyze_pic(uint16_t rgb[], uint8_t *img_buff_ptr)
{
	//analyze 320 pixels from the picture taken
	for(uint16_t i = 0; i < (IMAGE_BUFFER_SIZE*2); i+=4)
	{
		rgb[RED] += (uint16_t)img_buff_ptr[i/2]&0xF8;
		rgb[GREEN] += (uint16_t)(img_buff_ptr[i/2]&0x07)<<5 |
								(img_buff_ptr[i/2+1]&0xE0)>>3;
		rgb[BLUE] += (uint16_t)(img_buff_ptr[i/2+1]&0x1F)<<3;
	}

	//average the value for better results
	rgb[RED] /= (IMAGE_BUFFER_SIZE/2);
	rgb[GREEN] /= (IMAGE_BUFFER_SIZE/2);
	rgb[BLUE] /= (IMAGE_BUFFER_SIZE/2);
}

/*
*@brief			indicate the color seen based on experimental threshold
*				attention : white by default
*				set some RGB LEDS to match
*
*@param	rgb[]	table with colors red, green, blue
*@param	mean	value of the average from rgb
*
*/
void analyze_color(uint16_t rgb[], uint16_t mean)
{
	if(mean < COLOR_MIN_MEAN)
	{
		set_body_led(TRUE);
		color = BLACK;
	}
	else if((rgb[RED] > COLOR_THRESHOLD) && ((rgb[BLUE] < COLOR_THRESHOLD) &&
			(rgb[GREEN] < COLOR_THRESHOLD)))
	{
		toggle_rgb_led(LED2, RED_LED, TRUE);
		toggle_rgb_led(LED8, RED_LED, TRUE);
		color = RED;
	}
	else if((rgb[GREEN] > COLOR_THRESHOLD) && ((rgb[BLUE] < COLOR_THRESHOLD) &&
			(rgb[RED] < COLOR_THRESHOLD)))
	{
		toggle_rgb_led(LED2, GREEN_LED, TRUE);
		color = GREEN;
	}
	else if((rgb[BLUE] > COLOR_THRESHOLD) && ((rgb[RED] < COLOR_THRESHOLD) &&
			(rgb[GREEN] < COLOR_THRESHOLD)))
	{
		toggle_rgb_led(LED8, BLUE_LED, TRUE);
		color =  BLUE;
	}
	else
	{
		color = WHITE;
	}
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

	//deactivate auto white balance for camera (TRUE activate, FALSE deactivate)
	po8030_set_awb(FALSE);

	//camera settings to adjust depending on ambient light
	//brightness from -128 to 127 (default 0)
	//contrast from 0 to 255 (default 64)
	po8030_set_brightness(BRIGHTNESS);
	po8030_set_contrast(CONTRAST);



    while(TRUE)
    {
    	uint16_t distance;
    	//get the distance to object
    	 distance = VL53L0X_get_dist_mm();

    	 //picture taken only if at a turning distance
		 if((distance <= GOAL_DISTANCE + ERROR_THRESHOLD &&
			 distance >= GOAL_DISTANCE - ERROR_THRESHOLD) && rotation_finished())
		 {
			//starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
		 }
    }
}


static THD_WORKING_AREA(waProcessImage, 512);
static THD_FUNCTION(ProcessImage, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;

	uint16_t rgb[NB_COLOR]={0,0,0}, mean = 0, distance =0;

    while(TRUE)
    {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//get the distance to object
		 distance = VL53L0X_get_dist_mm();

		//color_ready is the signal that the color was analyzed (=1) or not (=0)
		 color_ready = FALSE;

		 //analyze the color of a picture only if at a turning distance
		 if((distance <= GOAL_DISTANCE + ERROR_THRESHOLD &&
			 distance >= GOAL_DISTANCE - ERROR_THRESHOLD) && rotation_finished())
		 {
			analyze_pic(rgb, img_buff_ptr);

			mean = (rgb[RED]+rgb[BLUE]+rgb[GREEN])/3;

			rgb[RED] = normalize(rgb[RED], mean);
			rgb[GREEN] = normalize(rgb[GREEN], mean);
			rgb[BLUE] = normalize(rgb[BLUE], mean);

			analyze_color(rgb, mean);

			color_ready = TRUE;
	        chThdSleepMilliseconds(SEC/4);
		 }
		 else
		 {
			 //turn off the lights only if not at turning distance
			 clear_leds();
			 set_body_led(FALSE);
		     chThdSleepMilliseconds(SEC/2);
		 }
    }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

colors get_color(void)
{
	return color;
}

uint8_t ready(void)
{
	return color_ready;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
