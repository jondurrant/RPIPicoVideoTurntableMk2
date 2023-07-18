/*
 * RotEncListener.cpp
 *
 * Abstract listener for the RotEnc
 *
 *  Created on: 24 Mar 2022
 *      Author: jondurrant
 */

#include <stdio.h>
#include "RotEncListener.h"

/***
 * Constructor
 */
RotEncListener::RotEncListener() {
	// NOP

}

/***
 * Destructor
 */
RotEncListener::~RotEncListener() {
	// NOP
}

/***
 * Notify of short Press
 * @param rotEnv - Ref to RotEnc in case multiple being used
 */
void RotEncListener::shortPress(void * rotEnc){
	printf("Short press\n");
}

/***
 * Notify of long Press
 * @param rotEnv - Ref to RotEnc in case multiple being used
 */
void RotEncListener::longPress(void * rotEnc){
	printf("Lond press\n");
}

/***
 * Notification of rotation
 * @param clockwise - Direction. True if clockwise
 * @param pos - Current possition (within 360)
 * @param rotEnc - Ref to RotEnc in case multiple being used
 */
void RotEncListener::rotate(bool clockwise, int16_t pos, void * rotEnc){
	if (clockwise){
		printf("Clockwise %d\n", pos);
	} else {
		printf("Withershins %d\n", pos);
	}
}
