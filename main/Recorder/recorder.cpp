/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include "recorder.hh"
#include <esp_log.h>

// Tag for logger
static const char* TAG_Recorder = "Recorder";

extern "C" void recorderTask(void * parameter) {
	ESP_LOGI(TAG_Recorder, "Recording task initiated");

}