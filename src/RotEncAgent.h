/*
 * RotEncAgent.h
 *
 * Rotary Encoder Switch Management
 * Samples state at 2ms intervals
 *
 *  Created on: 28 Mar 2022
 *      Author: jondurrant
 */

#ifndef ROTENCAGENT_H_
#define ROTENCAGENT_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Agent.h"
#include <stdint.h>
#include "hardware/gpio.h"
#include "pico/time.h"

#include "RotEncListener.h"
#include "SwitchMgr.h"


//Sampling time
#define POLLTIMEUS 2000


class RotEncAgent : public Agent, public SwitchObserver{
public:
	/***
	 * Constructor
	 * @param pGP - GPIO pin for the push switch
	 * @param aGP - GPIO pin for RotEnc A
	 * @param bGP - GPIO pin for RotEnc B
	 * @param numTicks - numTicks in 360 rotation
	 */
	RotEncAgent(uint8_t pGP, uint8_t aGP, uint8_t bGP,
			uint8_t numTicks);

	/***
	 * Destructor
	 */
	virtual ~RotEncAgent();


	/***
	 * Get current position of RotEnv
	 * @return
	 */
	int8_t getPos();

	/***
	 * Set a single listener to RotEnv events
	 * @param listener
	 */
	void setListener(RotEncListener *listener);


	/***
	 * Handle a short press from the switch
	 * @param gp - GPIO number of the switch
	 */
	virtual void handleShortPress(uint8_t gp);

	/***
	 * Handle a short press from the switch
	 * @param gp - GPIO number of the switch
	 */
	virtual void handleLongPress(uint8_t gp);

protected:
	/***
	 * Internal function to run the task from within the object
	 */
	void run();

	/***
	 * Internal function used by FreeRTOS to run the task
	 * @param pvParameters
	 */
	static void vTask( void * pvParameters );

	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();


	/***
	 * Handle rotate
	 * @param clockwise - true if clockwise, otherwise counter
	 */
	void handleRotate(bool clockwise);

	/***
	 * poll the RotEnv status
	 */
	void poll();



private:
	//Used for call back functions to find the object
	static RotEncAgent * pSelf;



	RotEncListener * pListener = NULL;

	uint8_t xPushGP;
	uint8_t xAGP;
	uint8_t xBGP;
	uint8_t xNumTicks=0;
	bool xA = false;
	bool xB = false;


	uint8_t xLast=0;
	int8_t xCount=0;

	int16_t xPos = 0;
	int16_t xDelta = 0;


	//These are lookups for the sequence to see in CW and CCW motion
	uint8_t xCW[4] ={2,0,3,1};
	uint8_t xCCW[4]={1,3,0,2};

	SwitchMgr *pSwitchMgr = NULL;

};

#endif /* ROTENCAGENT_H_ */
