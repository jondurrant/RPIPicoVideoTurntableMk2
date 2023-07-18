/*
 * UI.h
 *
 * Turntable UI on Oled, rotary encoder and Switches
 *
 *  Created on: 21 Feb 2023
 *      Author: jondurrant
 */

#ifndef SRC_UI_H_
#define SRC_UI_H_


#include "SwitchObserver.h"
#include "SwitchMgr.h"
#include "RotEncListener.h"
#include "RotEncAgent.h"
#include "Stepper28BYJ.h"

#include "Agent.h"
#include "OledDisplay.h"
#include <PicoLed.hpp>

#define NUM_SWITCHES 4
#define UI_MENU_ITEMS 7
#define MENU_TIME 100

/***
 * UI Class for the Turntable
 */
class UI : public SwitchObserver, public RotEncListener, public Agent {
public:
	/***
	 * Requires init functions to be called by client to setup
	 */
	UI();

	/***
	 * Deconstructor
	 */
	virtual ~UI();

	/***
	 * Setup the Rotary Encocder Agent
	 * @param agent - Rotary encoder agent
	 */
	void initRotEnc(RotEncAgent *agent);

	/***
	 * Setup the Switches
	 * @param sw1 - GPIO number
	 * @param sw2 - GPIO number
	 * @param sw3 - GPIO number
	 * @param sw4 - GPIO number
	 */
	void initSws(uint8_t sw1, uint8_t sw2, uint8_t sw3, uint8_t sw4);

	/***
	 * Set up the OLED board
	 * @param sda - GP for SDA
	 * @param sdl - GP for SDL
	 */
	void initOled(uint8_t sda, uint8_t sdl);

	/***
	 * Setup the stepper motor
	 * @param agent - Agent for driving the Stepper Motor
	 */
	void initStepper(Stepper28BYJ *agent);

	/***
	 * WS2812B LED Strip Controller
	 * @param p - pointer to the controller
	 */
	void initLeds(PicoLed::PicoLedController *p);

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

	/***
	 * Notify of short Press
	 * @param rotEnv - Ref to RotEnc in case multiple being used
	 */
	virtual void shortPress(void * rotEnc);

	/***
	 * Notify of long Press
	 * @param rotEnv - Ref to RotEnc in case multiple being used
	 */
	virtual void longPress(void * rotEnc);

	/***
	 * Notification of rotation
	 * @param clockwise - Direction. True if clockwise
	 * @param pos - Current possition (within 360)
	 * @param rotEnc - Ref to RotEnc in case multiple being used
	 */
	virtual void rotate(bool clockwise, int16_t pos, void * rotEnc);




protected:
	/***
	 * Start the preprogrammed sequence
	 * Move to start possition, then move to end possition at set speed in set direction
	 */
	void doStart();

	/***
	 * Stop all current movement, clear anything on the queue to do.
	 */
	void doStop();

	/***
	 * Do continuous rotation at speed in either CW or CCW direction
	 * @param cw - true if clockwise
	 */
	void doCont(bool cw);

	/***
	 * Reset. Stop the turntable, clear the queue of action and return to zero possition
	 */
	void doRst();

	/***
	 * Display the menu item on the OLED
	 */
	void menuDisplay();

	/***
	 * Display the turntable state on the OLED
	 */
	void stateDisplay();

	/***
	 * Display the edit item on the OLED
	 */
	void editDisplay();

	/***
	 * Undertake an edit of current item by the increment amount
	 * @param inc
	 */
	void doEdit(int inc);

	/***
	 * Move the turntable to the desired angle by the shortest route at max speed
	 * @param deg
	 */
	void shortestStepperMove(float deg);

	/***
	 * Task main run loop
	 */
	virtual void run();

	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
	RotEncAgent * pRotEnc = NULL;
	Stepper28BYJ *pStepper = NULL;
	SwitchMgr *pSwitch[NUM_SWITCHES];

	uint8_t xSw1, xSw2, xSw3, xSw4;
	uint8_t xOledGP;
	float xStart = 180.0;
	float xEnd = 90.0;
	float xSpeed = 3.0;
	bool xCW = false;


	//WS2812B LED parameters
	int16_t xHue = 100;
	int16_t xSat = 110;
	int16_t xBri = 120;


	//Menu and Edit controls
	bool xEditOn = false;
	int16_t xMenuTimer = 0;
	int8_t xMenuIdx = 0;

	OledDisplay xOledDisplay;
	// LED Strip
	PicoLed::PicoLedController *pStrip = NULL ;
};

#endif /* SRC_UI_H_ */
