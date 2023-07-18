/*
 * UI.cpp
 *
 *  Created on: 21 Feb 2023
 *      Author: jondurrant
 */

#include "UI.h"

/***
 * Requires init functions to be called by client to setup
 */
UI::UI() {
	// NOP

}

UI::~UI() {
	// NOP
}

/***
 * Setup the Rotary Encocder Agent
 * @param agent - Rotary encoder agent
 */
void UI::initRotEnc(RotEncAgent *agent){
	pRotEnc = agent;
	pRotEnc->setListener(this);
}

/***
 * Setup the Switches
 * @param sw1 - GPIO number
 * @param sw2 - GPIO number
 * @param sw3 - GPIO number
 * @param sw4 - GPIO number
 */
void UI::initSws(uint8_t sw1, uint8_t sw2, uint8_t sw3, uint8_t sw4){
	xSw1 = sw1;
	xSw2 = sw2;
	xSw3 = sw3;
	xSw4 = sw4;

	pSwitch[0]  = new SwitchMgr(xSw1);
	pSwitch[1]  = new SwitchMgr(xSw2);
	pSwitch[2]  = new SwitchMgr(xSw3);
	pSwitch[3]  = new SwitchMgr(xSw4);
	for (int i=0; i < NUM_SWITCHES; i++){
		pSwitch[i]->setObserver(this);
	}
}

/***
 * Set up the OLED board
 * @param sda - GP for SDA
 * @param sdl - GP for SDL
 */
void UI::initOled(uint8_t sda, uint8_t sdl){
	xOledDisplay.init(sda, sdl);
}

/***
 * Setup the stepper motor
 * @param agent - Agent for driving the Stepper Motor
 */
void UI::initStepper(Stepper28BYJ *agent){
	pStepper = agent;
}


/***
 * WS2812B LED Strip Controller
 * @param p - pointer to the controller
 */
void UI::initLeds(PicoLed::PicoLedController *p){
	pStrip = p;
	pStrip->fill( PicoLed::HSV(xHue, xSat, xBri) );
	pStrip->show();
}

/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void UI::handleShortPress(uint8_t gp){

	if(gp ==xSw1) {
		doStart();
	} else if (gp == xSw2) {
		doStop();
	} else if (gp == xSw3) {
		doCont(true);
	} else if (gp == xSw4) {
		doRst();
	}

}

/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void UI::handleLongPress(uint8_t gp){

	if(gp ==xSw1) {
		doStart();
	} else if (gp == xSw2) {
		doStop();
	} else if (gp == xSw3) {
		doCont(false);
	} else if (gp == xSw4) {
		doRst();
	}

}

/***
 * Notify of short Press
 * @param rotEnv - Ref to RotEnc in case multiple being used
 */
void UI::shortPress(void * rotEnc){
	xEditOn = ! xEditOn;
	xMenuTimer = MENU_TIME;
}

/***
 * Notify of long Press
 * @param rotEnv - Ref to RotEnc in case multiple being used
 */
void UI::longPress(void * rotEnc){
	xEditOn = ! xEditOn;
	xMenuTimer = MENU_TIME;
}

/***
 * Notification of rotation
 * @param clockwise - Direction. True if clockwise
 * @param pos - Current possition (within 360)
 * @param rotEnc - Ref to RotEnc in case multiple being used
 */
void UI::rotate(bool clockwise, int16_t pos, void * rotEnc){
	xMenuTimer = MENU_TIME;
	if (xMenuTimer > 0){
		if (! xEditOn){
			if (clockwise){
				xMenuIdx++;
				if (xMenuIdx >= UI_MENU_ITEMS){
					xMenuIdx = 0;
				}
			} else {
				xMenuIdx--;
				if (xMenuIdx < 0){
					xMenuIdx = UI_MENU_ITEMS -1;
				}
			}
		} else {
			if (clockwise){
				doEdit(+1);
			} else {
				doEdit(-1);
			}
		}
	}


}



/***
 * Start the preprogrammed sequence
 * Move to start possition, then move to end possition at set speed in set direction
 */
void UI::doStart(){
	if (pStepper != NULL){
		shortestStepperMove(xStart);
		pStepper->stepToDeg(xEnd, xSpeed, xCW);
	}
}

/***
 * Stop all current movement, clear anything on the queue to do.
 */
void UI::doStop(){
	if (pStepper != NULL){
		pStepper->stepHaltClear();
	}
}


/***
 * Do continuous rotation at speed in either CW or CCW direction
 * @param cw - true if clockwise
 */
void UI::doCont(bool cw){
	if (pStepper != NULL){
		pStepper->stepContinuous(xSpeed, cw);
	}
}

/***
 * Reset. Stop the turntable, clear the queue of action and return to zero possition
 */
void UI::doRst(){
	if (pStepper != NULL){
		pStepper->stepReset();
	}
}

/***
 * Display the menu item on the OLED
 */
void UI::menuDisplay(){
	char s[20];

	switch(xMenuIdx){
	case 0:
		sprintf(s, "%3.1f Deg", xStart);
		xOledDisplay.displayString("Start", s, 2);
		break;
	case 1:
		sprintf(s, "%3.1f Deg", xEnd);
		xOledDisplay.displayString("End", s, 2);
		break;
	case 2:
		if (xCW){
			xOledDisplay.displayString("Direction", "CW", 2);
		} else {
			xOledDisplay.displayString("Direction", "CCW", 2);
		}
		break;
	case 3:
		sprintf(s, "%1.1f RPM", xSpeed);
		xOledDisplay.displayString("Speed", s, 2);
		break;
	case 4:
		sprintf(s, "%d H", xHue);
		xOledDisplay.displayString("Hue", s, 2);
		break;
	case 5:
		sprintf(s, "%d S", xSat);
		xOledDisplay.displayString("Sat", s, 2);
		break;
	case 6:
		sprintf(s, "%d B", xBri);
		xOledDisplay.displayString("Bright", s, 2);
		break;
	}
}

/***
 * Display the edit item on the OLED
 */
void UI::editDisplay(){
	char s[20];

	switch(xMenuIdx){
	case 0:
		sprintf(s, "%3.1f Deg", xStart);
		xOledDisplay.displayString("E Start", s, 2);
		shortestStepperMove(xStart);
		break;
	case 1:
		sprintf(s, "%3.1f Deg", xEnd);
		xOledDisplay.displayString("E End", s, 2);
		shortestStepperMove(xEnd);
		break;
	case 2:
		if (xCW){
			xOledDisplay.displayString("E Direction", "CW", 2);
		} else {
			xOledDisplay.displayString("E Direction", "CCW", 2);
		}
		break;
	case 3:
		sprintf(s, "%1.1f RPM", xSpeed);
		xOledDisplay.displayString("E Speed", s, 2);
		break;
	case 4:
		sprintf(s, "%d H", xHue);
		xOledDisplay.displayString("E Hue", s, 2);
		break;
	case 5:
		sprintf(s, "%d S", xSat);
		xOledDisplay.displayString("E Sat", s, 2);
		break;
	case 6:
		sprintf(s, "%d B", xBri);
		xOledDisplay.displayString("E Bright", s, 2);
		break;
	}
}

/***
 * Undertake an edit of current item by the increment amount
 * @param inc
 */
void UI::doEdit(int inc){
	switch(xMenuIdx){
	case 0:
		xStart = xStart + inc;
		if (xStart > 360.0){
			xStart = 0.0;
		}
		if (xStart < 0.0){
			xStart = 360.0;
		}
		break;
	case 1:
		xEnd = xEnd + inc;
		if (xEnd > 360.0){
			xEnd = 0.0;
		}
		if (xEnd < 0.0){
			xEnd = 360.0;
		}
		break;
	case 2:
		xCW = ! xCW;
		break;
	case 3:
		xSpeed = xSpeed + ((float)inc / 10.0);
		if (xSpeed > 3.0){
			xSpeed = 3.0;
		}
		if (xSpeed < 0.1){
			xSpeed = 0.1;
		}
		break;
	case 4:
		//Rotate Hue
		xHue += inc;
		if (xHue > 255){
			xHue = 0;
		}
		if (xHue < 0){
			xHue = 255;
		}
		break;
	case 5:
		xSat += inc;
		if (xSat > 255){
			xSat = 255;
		}
		if (xSat < 0){
			xSat = 0;
		}
		break;
	case 6:
		xBri += inc;
		if (xBri > 255){
			xBri = 255;
		}
		if (xBri < 0){
			xBri = 0;
		}
		break;
	}
}


/***
 * Display the turntable state on the OLED
 */
void UI::stateDisplay(){
	char s[20];
	sprintf(s, "%3.1f Deg", pStepper->getDegrees());
	switch(pStepper->getState()){
	case StepperCmd:
		xOledDisplay.displayString("Turntable", s, 2);
		break;
	case StepperStop:
		xOledDisplay.displayString("Stop", s, 2);
		break;
	default:
		sprintf(s, "%d count", pStepper->getPos());
		xOledDisplay.displayString("Calib", s, 2);
		break;
	}
}


/***
 * Task main run loop
 */
void UI::run(){


	while (true){
		if (xMenuTimer > 0){
			if (xEditOn){
				editDisplay();
			} else {
				menuDisplay();
				xMenuTimer--;
			}
		} else {
			stateDisplay();
		}


		pStrip->fill( PicoLed::HSV(xHue, xSat, xBri) );
		pStrip->show();


		vTaskDelay(500);
	}

}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE UI::getMaxStackSize(){
	return 500;
}

/***
 * Move the turntable to the desired angle by the shortest route at max speed
 * @param deg
 */
void UI::shortestStepperMove(float deg){
	bool cw = true;
	float dif = 0.0;
	if (pStepper != NULL){
		float current = pStepper->getDegrees();
		if (current > deg){
			dif = current - deg;
			if ( dif < 180.0){
				cw = false;
			}
		} else {
			dif = deg - current;
			if ( dif > 180.0){
				cw = false;
			}
		}
		if (dif > 1.0){
			pStepper->stepHaltClear();
			pStepper->stepToDeg(deg, 3.0, cw);
		}

	}
}
