/* Author: NickTheBit
* Brief : Class manages and controls the 7 segment display
* designed to operate through a shift register.
*/

#include "segDisplay.h"

segDisplay::segDisplay() {
	// TODO: Search an spi channel for a shift register and connect to it
}

segDisplay * segDisplay::getInstance() {
	if (!instance) {
		instance = new segDisplay;
	}
	return instance;
}

segDisplay::~segDisplay() {
	// TODO: Deallocate resources if needed, remove deconstructor if not needed.
}