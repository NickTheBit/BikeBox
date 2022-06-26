/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "recorder.hh"
#include <esp_log.h>

// Tag for logger
static const char* TAG_Recorder = "Recorder";

extern "C" void recorderTask(void * parameter) {
	ESP_LOGI(TAG_Recorder, "Recording task initiated");

	while (1) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
	} // while
}