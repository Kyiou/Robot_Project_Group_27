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

//@brief	return color to be used in other files
colors get_color(void);

//@brief	return the signal that a color was analyzed
uint8_t ready(void);

//@brief init thread
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
