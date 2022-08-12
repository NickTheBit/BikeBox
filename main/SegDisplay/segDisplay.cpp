/* Author: NickTheBit
*  Brief : Class manages and controls the 7 segment display
*  designed to operate through a shift register.
*/

#include <esp_log.h>
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
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val, uint8_t valSize) {
	for (uint8_t i = 0; i < valSize; i++)  {
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

	/* todo: check if clearing the register is needed */
	gpio_set_level((gpio_num_t) SR_CLEAR_PIN, 0);
	gpio_set_level((gpio_num_t) SR_CLEAR_PIN, 1);

	/* todo: Improve flexibility by allowing normally off or on displays */
	uint8_t segmentConfiguration = 0;

	switch (displayedValue) {
	case ZERO:
		segmentConfiguration = (seg_a | seg_b | seg_c | seg_e | seg_f | seg_g);
		break;
	case ONE:
		segmentConfiguration = (seg_c | seg_f);
		break;
	case TWO:
		segmentConfiguration = (seg_a | seg_c | seg_d | seg_e | seg_g);
		break;
	case TRHEE:
		segmentConfiguration = (seg_a | seg_c | seg_d | seg_f | seg_g);
		break;
	case FOUR:
		segmentConfiguration = (seg_b | seg_d | seg_c | seg_f);
		break;
	case FIVE:
		segmentConfiguration = (seg_a | seg_b | seg_d | seg_f | seg_g);
		break;
	case SIX:
		segmentConfiguration = (seg_a | seg_b | seg_d | seg_e | seg_f | seg_g);
		break;
	case SEVEN:
		segmentConfiguration = (seg_a | seg_c | seg_f);
		break;
	case EIGHT:
		segmentConfiguration = (seg_a | seg_b | seg_c | seg_d | seg_e | seg_f | seg_g);
		break;
	case NINE:
		segmentConfiguration = (seg_a | seg_b | seg_c | seg_d | seg_f | seg_g);
		break;
	default:
		// Do nothing
		break;
	}

	shiftOut(SR_SER_PIN, SR_CLK_PIN, 0, segmentConfiguration, 8);
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