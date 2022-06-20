#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_spi_flash.h>
#include <esp_chip_info.h>

#include "System/system.hh"
#include "UserInterface/userInterface.hh"
#include "Recorder/recorder.hh"

static const char* TAG_Main = "Main";

extern "C" void app_main(void) {

    /* Main is to initiate three tasks
     * 1. System task, will handle updates, configuration, and data export procedures.
     * 2. Interface task, will handle rendering on the screen and user input
     * 3. Recorder task, it has the highest priority responsible for talking to the sensors and storing the data.
     */

	/* Displaying Chip info */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI(TAG_Main,
			"This is %s chip with %d CPU core(s), WiFi%s%s, ",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	ESP_LOGI(TAG_Main,"silicon revision %d, ", chip_info.revision);
	ESP_LOGI(TAG_Main,"%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
	       (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	ESP_LOGI(TAG_Main,"Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());


    xTaskCreate( systemTask, "System", 2048, nullptr, tskIDLE_PRIORITY , nullptr);
    xTaskCreate( UITask, "UserInterface", 2048, nullptr, tskIDLE_PRIORITY, nullptr);
    xTaskCreate( recorderTask, "Recorder", 2048, nullptr, tskIDLE_PRIORITY, nullptr);

	ESP_LOGI(TAG_Main, "Tasks Created successfully.");
}


