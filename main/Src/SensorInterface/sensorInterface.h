//
// Created by nick on 12/04/2022.
//

#ifndef BIKEBOX_SENSORINTERFACE_H
#define BIKEBOX_SENSORINTERFACE_H

#include <stdint.h>
#include <esp_log.h>

class sensorInterface{
public:
	sensorInterface();
	sensorInterface getInstance();
	uint8_t initAll();
	~sensorInterface();

private:
	sensorInterface *instance;
};


#endif //BIKEBOX_SENSORINTERFACE_H
