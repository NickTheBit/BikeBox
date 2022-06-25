/* Author: NickTheBit
 * Brief : Class manages and controls the 7 segment display
 * designed to operate through a shift register.
 */

#pragma once

// Utility functions and enums

typedef enum {
	LSBFIRST = 0,
	LSBLAST
} bitOrder_t;

// todo: move utility functions in a universally accessible header.
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);

	// Singleton class
class segDisplay {
	segDisplay * instance = nullptr;

	segDisplay();
	~segDisplay();

  public:
	segDisplay * getInstance(void);

};
