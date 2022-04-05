/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <recorder.hh>
#include <esp_log.h>

#include <lvgl.h>

extern "C" void recorderTask(void * parameter) {
	
    ESP_LOGI(TAG_Recorder, "Recording task initiated");

    lv_init();
    // Main system loop
    while (1) {
        lv_tick_inc(10);
        vTaskDelay(10);
    }
}
