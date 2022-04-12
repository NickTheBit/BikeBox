/* File: userInterface.cpp
 * Author: Nick Gkloumpos
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <UserInterface.hh>

extern "C" void UITask(void * parameter) {

    ESP_LOGI(TAG_Gui, "Gui task initialized");

    // User Interface main loop
    while (1) {

        vTaskDelay(1050 / portTICK_PERIOD_MS);
    }
}
