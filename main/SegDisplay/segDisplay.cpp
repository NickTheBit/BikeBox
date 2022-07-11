/* Author: NickTheBit
*  Brief : Class manages and controls the 7 segment display
*  designed to operate through a shift register.
*/

#include "segDisplay.h"

segDisplay * segDisplay::instance = nullptr;

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
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
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

	// Flushing to main registers
	gpio_set_level((gpio_num_t) SR_REGISTER_PIN, 1);
	gpio_set_level((gpio_num_t) SR_REGISTER_PIN, 0);
}

segDisplay::segDisplay() {
	// Setting assigned pins to output
	gpio_set_direction((gpio_num_t)SR_SER_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction((gpio_num_t)SR_CLK_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction((gpio_num_t)SR_OUTPUT_ENABLE_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction((gpio_num_t)SR_CLEAR_PIN, GPIO_MODE_OUTPUT);
}

segDisplay * segDisplay::getInstance() {
	if (!instance) {
		instance = new segDisplay;
	}
	return instance;
}

/* Sets the digit to be displayed by the display returns true if it succeeds. */
bool segDisplay::setDigit(sevSegDigit_t digit) {
	displayedValue = digit;

	gpio_set_level((gpio_num_t) SR_CLEAR_PIN, 1);

	switch (displayedValue) {
	case ZERO:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b10000000);
		break;
	case ONE:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b11010111);
		break;
	case TWO:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b01100000);
		break;
	case TRHEE:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b01000100);
		break;
	case FOUR:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b00010110);
		break;
	case FIVE:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b00001100);
		break;
	case SIX:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b00001000);
		break;
	case SEVEN:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b11000111);
		break;
	case EIGHT:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b00000000);
		break;
	case NINE:
		shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, 0b00000100);
		break;
	default:
		// Do nothing
		break;
	}
	return true;
}

sevSegDigit_t segDisplay::getDigit() {
	return displayedValue;
}

bool segDisplay::enableDisplay() {
	esp_err_t error = gpio_set_level((gpio_num_t) SR_OUTPUT_ENABLE_PIN, 0);
	if (error == ESP_OK) {
		return true;
	} else {
		return false;
	}
}

bool segDisplay::disableDisplay() {
	esp_err_t error = gpio_set_level((gpio_num_t) SR_OUTPUT_ENABLE_PIN, 1);
	if (error == ESP_OK) {
		return true;
	} else {
		return false;
	}
}

segDisplay::~segDisplay() {
	setDigit(NAN);
	// TODO: Deallocate resources if needed, remove deconstructor if not needed.
}