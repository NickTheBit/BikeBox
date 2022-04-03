/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <recorder.hh>
#include <esp_log.h>

extern "C" void recorderTask(void * parameter) {

    ESP_LOGI(TAG_Recorder, "Recording task initiated");

    // Main system loop
    while (1) {
        vTaskDelay(10);
    }
}
