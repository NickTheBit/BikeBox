/* File: system.cpp
 * Author: Nick Gkloumpos
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "system.hh"
#include <esp_log.h>

extern "C" void systemTask(void * parameter) {

    ESP_LOGI(TAG_System, "System task initiated");

    // Main system loop
    while (1) {
	    vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
