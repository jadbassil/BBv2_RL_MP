#ifndef __HW_LED_H__
#define __HW_LED_H__

//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// define
//------------------------------------------------------------------------------
typedef enum __packed {
	RED, ORANGE, BLUE, GREEN, CYAN, YELLOW, PURPLE, GREY, WHITE,
	DARK_RED, DARK_ORANGE, BROWN, DARK_GREEN, DARK_BLUE, DARK_CYAN, DARK_PURPLE, DARK_GREY,
	BLACK, NB_COLORS
} ColorName;

typedef struct _Color {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} Color;

void setColor(uint8_t color_index);

//------------------------------------------------------------------------------
// prototype
//------------------------------------------------------------------------------
void setHWLED(unsigned char ucRedPower, unsigned char ucGreenPower, unsigned char ucBluePower);

#endif
