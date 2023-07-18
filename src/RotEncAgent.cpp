/*
 * RotEncAgent.cpp
 *
 * Rotary Encoder Switch Management
 * Samples state at 2ms intervals
 *  Created on: 28 Mar 2022
 *      Author: jondurrant
 */

#include "RotEncAgent.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

/***
 * Used by callback static functions
 */
RotEncAgent * RotEncAgent::pSelf = NULL;

/***
 * Constructor
 * @param pGP - GPIO pin for the push switch
 * @param aGP - GPIO pin for RotEnc A
 * @param bGP - GPIO pin for RotEnc B
 * @param numTicks - numTicks in 360 rotation
 */
RotEncAgent::RotEncAgent(uint8_t pGP, uint8_t aGP, uint8_t bGP,
		uint8_t numTicks) {
	xPushGP = pGP;
	xAGP=aGP;
	xBGP=bGP;
	xNumTicks = numTicks;
	RotEncAgent::pSelf = this;


	//Set Push Switch
	pSwitchMgr = new SwitchMgr(xPushGP);
	pSwitchMgr->setObserver(this);

	//A
	gpio_init(xAGP);
	gpio_set_dir(xAGP, GPIO_IN);
	gpio_pull_up (xAGP);


	//A
	gpio_init(xBGP);
	gpio_set_dir(xBGP, GPIO_IN);
	gpio_pull_up (xBGP);


	xLast = gpio_get(xAGP);
	xLast = xLast << 1;
	xLast = (gpio_get(xBGP)) | xLast;
}

/***
 * Destructor
 */
RotEncAgent::~RotEncAgent() {
	// NOP
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE RotEncAgent::getMaxStackSize(){
	return 500;
}

/***
 * Internal function used by FreeRTOS to run the task
 * @param pvParameters
 */
void RotEncAgent::vTask( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below.
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
    */
	RotEncAgent *task = (RotEncAgent *) pvParameters;
	task->run();
}

/***
 * Internal function to run the task from within the object
 */
void RotEncAgent::run(){
	for (;;){
		poll();
		vTaskDelay(2);
	}

}


/***
 * Process short push
 */

void RotEncAgent::handleShortPress(uint8_t gp){
	//printf("Short Press\n");
	if (pListener != NULL){
		pListener->shortPress((void *)this);
	}
}

/***
 * Process long press
 */
void RotEncAgent::handleLongPress(uint8_t gp){
	//printf("Long Press\n");
	if (pListener != NULL){
		pListener->longPress((void *)this);
	}
}

/***
 * Handle rotate
 * @param clockwise - true if clockwise, otherwise counter
 */
void RotEncAgent::handleRotate(bool clockwise){
	//printf("Long Press\n");
	if (pListener != NULL){
		pListener->rotate(clockwise, xPos, (void *)this);
	}
}



/***
 * poll the RotEnv status
 */
void RotEncAgent::poll(){
	uint8_t c;
	c = gpio_get(xAGP);
	c = c << 1;
	c = (gpio_get(xBGP)) | c;


	if (xCW[xLast] == c){
		xCount++;
		if (xCount > 3){
			xPos++;
			if (xPos == xNumTicks){
				xPos = 0;
			}
			//printf("Clockwise %d %d\n", xPos, xCount);
			handleRotate(true);
			xCount = 0;
		}

		xLast = c;
	}

	if (xCCW[xLast] == c){
		xCount-- ;
		if (xCount < -3){
			xPos--;
			if (xPos == -1){
				xPos = xNumTicks - 1;
			}
			//printf("Withershins %d %d\n", xPos, xCount);
			handleRotate(false);
			xCount = 0;
		}

		xLast = c;
	}

}

/***
 * Get current position of RotEnv
 * @return
 */
int8_t RotEncAgent::getPos(){
	return xPos;
}

/***
 * Set a single listener to RotEnv events
 * @param listener
 */
void RotEncAgent::setListener(RotEncListener *listener){
	pListener = listener;
}
