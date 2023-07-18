/*
 * RotEnvListener.h
 *
 * Abstract listener for the RotEnc
 *
 *  Created on: 24 Mar 2022
 *      Author: jondurrant
 */

#ifndef ROTENC_SRC_ROTENCLISTENER_H_
#define ROTENC_SRC_ROTENCLISTENER_H_

#include <stdint.h>

class RotEncListener {
public:
	/***
	 * Constructor
	 */
	RotEncListener();

	/***
	 * Destructor
	 */
	virtual ~RotEncListener();

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

};

#endif /* ROTENC_SRC_ROTENCLISTENER_H_ */
