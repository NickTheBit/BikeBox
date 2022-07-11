/* File: userInterface.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>

#include "userInterface.hh"

static const char* TAG_Gui = "GUI";

extern "C" void UITask(void * parameter) {

    ESP_LOGI(TAG_Gui, "Gui task initialized");

	segDisplay::getInstance()->enableDisplay();

    // User Interface main loop
    while (1) {
		for (uint8_t i = 0; i < 10; i++) {
			segDisplay::getInstance()->setDigit((sevSegDigit_t)i);
			vTaskDelay(250 / portTICK_PERIOD_MS);
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
    } // while
}
