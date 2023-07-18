/*
 * StepperObserver.h
 *
 * Observer for the Stepper motor actions
 *
 *  Created on: 29 Dec 2022
 *      Author: jondurrant
 */

#ifndef DEMO_28BYJ_28_SRC_STEPPEROBSERVER_H_
#define DEMO_28BYJ_28_SRC_STEPPEROBSERVER_H_

#include "pico/stdlib.h"
#include <stdio.h>

class StepperObserver {
public:
	StepperObserver();
	virtual ~StepperObserver();

	/***
	 * Callback when the requested action completed.
	 * @param actionNum - Action number of the requested action that has completed.
	 */
	virtual void actionComplete(uint32_t actionNum) = 0;
};

#endif /* DEMO_28BYJ_28_SRC_STEPPEROBSERVER_H_ */
