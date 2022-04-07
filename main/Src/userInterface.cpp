/* File: userInterface.cpp
 * Author: Nick Gkloumpos
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <UserInterface.hh>
#include <lvgl/lvgl.h>

extern "C" void UITask(void * parameter) {

	lv_init();
    ESP_LOGI(TAG_Gui, "Gui task initialized");

    // User Interface loop
    while (1) {

        vTaskDelay(10);
    }
}
