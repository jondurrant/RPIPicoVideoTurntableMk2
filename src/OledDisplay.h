/*
 * OledDisplay.h
 *
 * Oled display driver
 *
 *  Created on: 16 Mar 2022
 *      Author: jondurrant
 */

#ifndef OLEDDISPLAY_H_
#define OLEDDISPLAY_H_

#include <stdint.h>
extern "C" {
#include "ssd1306.h"
}

class OledDisplay {
public:
	OledDisplay();

	/***
	 * Constructor
	 * @param sda - GP number for the SDA pin
	 * @param sdl - GP number for the SDL pin
	 */
	OledDisplay(uint8_t sda, uint8_t sdl);

	/***
	 * Destructor
	 */
	virtual ~OledDisplay();

	/***
	 * Init
	 * @param sda - GP number for the SDA pin
	 * @param sdl - GP number for the SDL pin
	 */
	void init(uint8_t sda, uint8_t sdl);

	/***
	 * Display two lines of string on the display
	 * @param l1 = String 1 for top line
	 * @param l2 - String 2 for the bottom line
	 * @param scale - Scale size. tend to use 2
	 */
	void displayString(const char *l1, const char *l2, uint8_t scale=1);

protected:
	ssd1306_t xDisp;
	i2c_inst_t *pI2C =  i2c0;
	char xBuf[80];

};

#endif /* OLEDDISPLAY_H_ */
