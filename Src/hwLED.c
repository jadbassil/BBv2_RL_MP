//------------------------------------------------------------------------------
// include
//------------------------------------------------------------------------------
#include "stm32f0xx_hal.h"
#include <hwLED.h>
//------------------------------------------------------------------------------
// define
//------------------------------------------------------------------------------
#define LED_ADDR	0xCC
#define MAX_CURRENT	39 // =20mA

extern I2C_HandleTypeDef hi2c1;

//enum color_names {RED, ORANGE, YELLOW, DUNNO, CYAN, LIGHT_BLUE, DEEP_BLUE};
//uint8_t colors[7][3] = {{50,0,0}, {50,25,0}, {50,50,0}, {15,50,25}, {0,50,50}, {0,25,50}, {0,0,50}};

//RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, GREY, WHITE
//Color colors[9] = {
//		{0, 0, 0}, {50, 0, 0}, {0, 50, 0}, {0, 0, 50}, {50, 50, 0},
//		{50, 0, 50}, {0, 50, 50}, {20, 20, 20}, {50, 50, 50}
//};


Color colors[NB_COLORS] = {
		{60, 0, 0}, {60, 30, 0}, {0, 0, 60} , {0, 60, 0}, {0, 60, 60},
		{60, 60, 0}, {47, 16, 57}, {30, 30, 30}, {60, 60, 60},
		{20, 0, 0}, {20, 10, 0}, {100, 50, 0}, {0, 20, 0}, {0, 20, 20},
		{0, 0, 20}, {16, 5, 19}, {8, 8, 8}, {0, 0, 0}};

void setColor(uint8_t color_index) {
	uint8_t idx = color_index%9;
	Color *color = &colors[idx];
	setHWLED(color->r, color->g, color->b);
}

//------------------------------------------------------------------------------
// void setHWLED(unsigned char ucRedPower, unsigned char ucGreenPower, unsigned char ucBluePower)
// updated by bpiranda, 2018-06-05
//------------------------------------------------------------------------------
void setHWLED(unsigned char ucRedPower, unsigned char ucGreenPower, unsigned char ucBluePower) {
    static unsigned char ucPreviousRedPower = 101; // impossible value to start with a new color
    static unsigned char ucPreviousGreenPower = 101;
    static unsigned char ucPreviousBluePower = 101;
    static unsigned char ucPreviousReg = 0xFF; // impossible value to turn ON/OFF on start
    unsigned char tab_uc[2] = {0,0};
    unsigned char ucEnReg=(ucPreviousReg==0xFF)?0x3F:ucPreviousReg;


    // Maximum Control
    if (ucRedPower > 100) ucRedPower = 100;
    if (ucGreenPower > 100) ucGreenPower = 100;
    if (ucBluePower > 100) ucBluePower = 100;

//---------------------------------------------------------------------------------------------------------------------
//ATTENTION LE MODE 0 -> ROUGE = 1 BLEU = 0 VERT = 2
//Inversement des bits par rapport au schéma les 2 bits de poids faibles seront donc pour le bleu et non le rouge !!
//---------------------------------------------------------------------------------------------------------------------
    // RED
    if (ucRedPower != ucPreviousRedPower) {
        ucPreviousRedPower=ucRedPower;
        if (ucRedPower==0) {
          ucEnReg &= 0x33;
        } else {
            ucEnReg |= 0x0C;
            // register adress for RED component
            tab_uc[0] = 1;
            tab_uc[1] = (unsigned char)(((int)(ucRedPower-1) * MAX_CURRENT)/99);
            HAL_I2C_Master_Transmit(&hi2c1, LED_ADDR, &tab_uc[0], sizeof(tab_uc), 500);
        }
    }
    // GREEN, warning C1 && C2 connectors
    if (ucGreenPower != ucPreviousGreenPower) {
        ucPreviousGreenPower=ucGreenPower;
        if (ucGreenPower==0) {
            ucEnReg &= 0x0F;
        } else {
            ucEnReg |= 0x30;
            // register adress for GREEN component
            tab_uc[0] = 2; // ports C
            tab_uc[1] = (unsigned char)(((int)(ucGreenPower-1) * MAX_CURRENT)/99);
            HAL_I2C_Master_Transmit(&hi2c1, LED_ADDR, &tab_uc[0], sizeof(tab_uc), 500);
        }
    }
    // BLUE, warning B1 && B2 connectors
    if (ucBluePower != ucPreviousBluePower) {
        ucPreviousBluePower = ucBluePower;
        if (ucBluePower==0) {
            ucEnReg &= 0x3C;
        } else {
            ucEnReg |= 0x03;
            // register adress for BLUE component
            tab_uc[0] = 0; // ports C
            tab_uc[1] = (unsigned char)(((int)(ucBluePower-1) * MAX_CURRENT)/99);
            HAL_I2C_Master_Transmit(&hi2c1, LED_ADDR, &tab_uc[0], sizeof(tab_uc), 500);
        }
    }
    // Turn ON/OFF LEDs
    if (ucEnReg!=ucPreviousReg) {
        ucPreviousReg = ucEnReg;
        tab_uc[0] = 3; // ports ON/OFF
        tab_uc[1] = ucEnReg;
        HAL_I2C_Master_Transmit(&hi2c1, LED_ADDR, &tab_uc[0], sizeof(tab_uc), 500);
    }
}

