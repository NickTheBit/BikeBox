/* Author: NickTheBit
 * Brief : Class manages and controls the 7 segment display
 * designed to operate through a shift register.
 */

#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Utility functions and enums
typedef enum {
	LSBFIRST = 0,
	LSBLAST
} bitOrder_t;

// todo: move utility functions in a universally accessible header.
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);

// Pin definitions for 7 segment
#define SR_OUTPUT_ENABLE_PIN    26
#define SR_CLEAR_PIN            25
#define SR_SER_PIN              27
#define SR_CLK_PIN              33
#define SR_REGISTER_PIN         32
//todo: Move pin configuration to menuconfig.

// Segment enumeration
typedef enum {
	seg_a = 0x80,
	seg_b = 0x40,
	seg_c = 0x20,
	seg_d = 0x10,
	seg_e = 0x08,
	seg_f = 0x04,
	seg_g = 0x02,
	seg_n = 0x01
} segments_t;

// Seven segment configuration enumerations
typedef enum {
	ZERO = 0,
	ONE,
	TWO,
	TRHEE,
	FOUR,
	FIVE,
	SIX,
	SEVEN,
	EIGHT,
	NINE,
	NAN
} sevSegDigit_t;

// Singleton class
class segDisplay {
	static segDisplay * instance;
	sevSegDigit_t displayedValue = NAN;

	segDisplay();
	~segDisplay();

  public:
	static segDisplay * getInstance();
	bool setDigit(sevSegDigit_t);
	sevSegDigit_t getDigit();
	bool enableDisplay();
	bool disableDisplay();

};
