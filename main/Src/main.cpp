#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>

#include <system.hh>

static const char* TAG_Main = "Main";

extern "C" void app_main(void) {

    /* Main is to initiate three tasks
     * 1. System task, will handle updates, configuration, and data export procedures.
     * 2. Interface task, will handle rendering on the screen and user input
     * 3. Recorder task, highest priority responsible for talking to the sensors and storing the data.
     */

    ESP_LOGI(TAG_Main, "Main process started");

    TaskHandle_t systemHandle;
    xTaskCreate( systemTask, "System", 2048, nullptr, tskIDLE_PRIORITY , &systemHandle);

}
