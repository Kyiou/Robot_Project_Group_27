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

//return color
colors rgb_analyze(void);

void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
