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

//return color to be used in other files
colors get_color(void);

//return the signal that a color was analyzed
uint8_t ready(void);

void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
