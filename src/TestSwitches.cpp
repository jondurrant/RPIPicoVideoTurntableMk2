/*
 * TestSwitches.cpp
 *
 *  Created on: 10 Feb 2023
 *      Author: jondurrant
 */

#include "TestSwitches.h"

#include "stdio.h"

TestSwitches::TestSwitches() {

	pSwitch[0] = new SwitchMgr(6);
	pSwitch[1] = new SwitchMgr(7);
	pSwitch[2] = new SwitchMgr(8);
	pSwitch[3] = new SwitchMgr(9);

	//pSwitch[4] = new SwitchMgr(28);
/*
	pSwitch[4] = new SwitchMgr(3);
	pSwitch[5] = new SwitchMgr(4);
	pSwitch[6] = new SwitchMgr(5);
	*/


	for (int i=0; i < NUM_SWITCHES; i++){
		pSwitch[i]->setObserver(this);
	}

}

TestSwitches::~TestSwitches() {
	// TODO Auto-generated destructor stub
}


/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void TestSwitches::handleShortPress(uint8_t gp){
	printf("Short Press on %d\n", gp);

}

/***
 * Handle a short press from the switch
 * @param gp - GPIO number of the switch
 */
void TestSwitches::handleLongPress(uint8_t gp){
	printf("Long Press on %d\n", gp);
}
