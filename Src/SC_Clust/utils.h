/*
 * utils.h
 *
 *  Created on: Oct 5, 2020
 *      Author: jad
 */

#ifndef SC_CLUST_UTILS_H_
#define SC_CLUST_UTILS_H_

#include <math.h>

typedef enum portReferences { DOWN, NORTH, EAST, WEST, SOUTH, UP, NUM_PORTS, UNDEFINED_PORT = 255 }  enum___portReferences;

typedef struct Coord {
    uint8_t x;
    uint8_t y;
    uint8_t z;
} Coord;

uint8_t neighborCount() {
    uint8_t nb=0;

    for (uint8_t p=0; p<NB_SERIAL_PORT; ++p) {
        if (is_connected(p)) {
            nb++;
        }
    }
    return nb;
}

void delay(int dt) {
    uint32_t end_t = HAL_GetTick()+dt;
    while (HAL_GetTick() < end_t) {
    }
}

float euclDist(Coord c1, Coord c2){
    return sqrt(    (c1.x-c2.x)*(c1.x-c2.x) +
                    (c1.y-c2.y)*(c1.y-c2.y) +
                    (c1.z-c2.z)*(c1.z-c2.z)
                );
}

#endif /* SC_CLUST_UTILS_H_ */
