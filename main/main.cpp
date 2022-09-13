#include "Recorder/recorder.hh"
#include "System/system.hh"
#include "UserInterface/userInterface.hh"

#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <spi_flash_mmap.h>

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

	uint32_t size_flash_chip;
	esp_flash_get_size(NULL, &size_flash_chip);

	ESP_LOGI(TAG_Main,"%dMB %s flash", (int) size_flash_chip / (1024 * 1024),
	       (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	ESP_LOGI(TAG_Main,"Minimum free heap size: %u bytes", (unsigned int) esp_get_minimum_free_heap_size());


    xTaskCreate( systemTask, "System", 2048, nullptr, tskIDLE_PRIORITY , nullptr);
    xTaskCreate( UITask, "UserInterface", 2048, nullptr, tskIDLE_PRIORITY, nullptr);
    xTaskCreate( recorderTask, "Recorder", 2048, nullptr, tskIDLE_PRIORITY, nullptr);

	ESP_LOGI(TAG_Main, "Tasks Created successfully.");
}


