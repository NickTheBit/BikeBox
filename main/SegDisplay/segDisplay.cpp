/* Author: NickTheBit
* Brief : Class manages and controls the 7 segment display
* designed to operate through a shift register.
*/

#include <cstdint>
#include <driver/gpio.h>
#include "segDisplay.h"

/*
 * Gradually reads data from pin and shoves it in an 8 bit value
 * Assumes all pin-modes are set properly.
 */
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		// Setting clock high
		gpio_set_level((gpio_num_t) clockPin, 1);

		if (bitOrder == LSBFIRST)
			value |= gpio_get_level((gpio_num_t) dataPin) << i;
		else
			value |= gpio_get_level((gpio_num_t) dataPin) << (7 - i);

		// Setting clock low
		gpio_set_level((gpio_num_t) clockPin, 0);
	}
	return value;
}

/* Gradually writes data to a pin and shoves it in an 8 bit value
 * Assumes all pin-modes are set properly, manhandles the clock */
void shiftOut(uint8_t dataPin, uint8_t clockPin, bitOrder_t bitOrder, uint8_t val) {
	for (uint8_t i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST) {
			gpio_set_level((gpio_num_t) dataPin, val & 1);
			val >>= 1;
		} else {
			gpio_set_level((gpio_num_t) dataPin, (val & 128) != 0);
			val <<= 1;
		}

		// Advancing the clock
		gpio_set_level((gpio_num_t) clockPin, 1);
		gpio_set_level((gpio_num_t) clockPin, 0);
	}
}

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