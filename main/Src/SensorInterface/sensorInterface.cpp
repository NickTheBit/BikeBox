//
// Created by nick on 12/04/2022.
//

#include "sensorInterface.h"

const char* TAG_SensorInterface = "Sensor Interface";

sensorInterface::sensorInterface() {

}

sensorInterface sensorInterface::getInstance() {
	if (!instance) {
		instance = new sensorInterface();
	}
	return *instance;
}

uint8_t sensorInterface::initAll() {
	// todo: Implement holistic initialization sequence
	return 1;
}

sensorInterface::~sensorInterface() {

}
