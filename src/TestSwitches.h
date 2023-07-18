/*
 * TestSwitches.h
 *
 *  Created on: 10 Feb 2023
 *      Author: jondurrant
 */

#ifndef SRC_TESTSWITCHES_H_
#define SRC_TESTSWITCHES_H_

#include "SwitchObserver.h"
#include "SwitchMgr.h"

#define NUM_SWITCHES 4

class TestSwitches : public SwitchObserver {
public:
	TestSwitches();
	virtual ~TestSwitches();

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

private:
	SwitchMgr *pSwitch[NUM_SWITCHES];
};

#endif /* SRC_TESTSWITCHES_H_ */
