/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <recorder.hh>
#include <esp_log.h>

extern "C" void recorderTask(void * parameter) {
    ESP_LOGI(TAG_Recorder, "Recording task initiated");

}
