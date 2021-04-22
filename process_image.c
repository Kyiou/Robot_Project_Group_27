#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

/*
static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
*/


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
/*
color extract_color(uint8_t *buffer)
{
	uint16_t mean_red = 0, mean_green = 0, mean_blue = 0;

	for(uint16_t i = 0 ; i < 2 * IMAGE_BUFFER_SIZE ; i+=4)
	{
			mean_red += (buffer[i/2] & 0xF8);

			mean_blue += (buffer[i/2 + 1] & 0x1F) << 3;

			mean_green += ((buffer[i/2] & 0x07) << 5 | (buffer[i/2 + 1] & 0xE0) >> 3);
	}

	mean_red /= (IMAGE_BUFFER_SIZE/2);
	mean_blue /= (IMAGE_BUFFER_SIZE/2);
	mean_green /= (IMAGE_BUFFER_SIZE/2);

	if((mean_red > mean_blue) && (mean_red > mean_green))
	{
		chprintf((BaseSequentialStream *)&SDU1, "red\r\n");
		return red;
	}

	if((mean_blue > mean_red) && (mean_blue > mean_green))
	{
		chprintf((BaseSequentialStream *)&SDU1, "blue\r\n");
		return blue;
	}

	if((mean_green > mean_blue) && (mean_green > mean_red))
	{
		chprintf((BaseSequentialStream *)&SDU1, "green\r\n");
		return green;
	}

	chprintf((BaseSequentialStream *)&SDU1, "black\r\n");
	return black;
}
*/

/*
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}
*/

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	//uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	//uint16_t lineWidth = 0;

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		/*

		//Extracts the colors
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=4)
		{
			//extracts first 5 bits of red and 3 of green
			image[i/2] = (uint8_t)img_buff_ptr[i];

			//extracts 3 bits of green and 5 of blue
			image[(i/2)+1] = (uint8_t)img_buff_ptr[i+1];
		}
		*/

	    uint16_t r = 0, g = 0, b = 0;

	    for(uint16_t i = 0; i < (IMAGE_BUFFER_SIZE*2); i+=4)
	    {
	    	r += (int)img_buff_ptr[i/2]&0xF8;
	    	g += (int)(img_buff_ptr[i/2]&0x07)<<5 | (img_buff_ptr[i/2+1]&0xE0)>>3;
	    	b += (int)(img_buff_ptr[i/2+1]&0x1F)<<3;
	    }

	    r /= (IMAGE_BUFFER_SIZE/2);
	    g /= (IMAGE_BUFFER_SIZE/2);
	    b /= (IMAGE_BUFFER_SIZE/2);
/*
		r = (int)img_buff_ptr[0]&0xF8;
        g = (int)(img_buff_ptr[0]&0x07)<<5 | (img_buff_ptr[1]&0xE0)>>3;
        b = (int)(img_buff_ptr[1]&0x1F)<<3;
 */
        chprintf((BaseSequentialStream *)&SDU1, "CAMERA\r\n");
        chprintf((BaseSequentialStream *)&SDU1, "R=%3d, G=%3d, B=%3d\r\n\n", r, g, b);

/*
		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
*/
    }
}

/*
float get_distance_cm(void)
{
	return distance_cm;
}
*/
/*
uint16_t get_line_position(void){
	return line_position;
}
*/

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
