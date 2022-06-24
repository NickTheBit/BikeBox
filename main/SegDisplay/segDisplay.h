/* Author: NickTheBit
 * Brief : Class manages and controls the 7 segment display
 * designed to operate through a shift register.
 */

#pragma once

// Singleton class
class segDisplay {
	segDisplay * instance = nullptr;
	

	segDisplay();
	~segDisplay();

  public:
	segDisplay * getInstance(void);

};
