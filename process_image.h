#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

typedef enum
{
	RED = 0,
	GREEN,
	BLUE,
	BLACK,
	WHITE,
}colors;


/*
*@brief						return color to be used in other files
*
*@retval	color			the color analyzed from the pictures taken
*/
colors get_color(void);


/*
*@brief						return the signal that a color was analyzed
*
*@retval	color_ready		signal that the color is ready (!=0) or not (=0)
*/
uint8_t ready(void);


/*
*@brief						init thread
*/
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
