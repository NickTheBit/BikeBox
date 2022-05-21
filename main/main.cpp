#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_spi_flash.h>
#include <hal/gpio_hal.h>
#include <driver/gpio.h>
#include <cstdint>
#include <freertos/queue.h>

#include "System/system.hh"
#include "UserInterface/userInterface.hh"
#include "Recorder/recorder.hh"

static const char* TAG_Main = "Main";
static QueueHandle_t qSensorReadEvent = NULL;

extern "C" void IRAM_ATTR gpio_isr_handler(void* arg) {
	auto gpio_num = (long int) arg;
	if(gpio_num == GPIO_NUM_18) {
		uint8_t qSignal = 1; // dummy signal value for the queue
		xQueueSendFromISR(qSensorReadEvent, &qSignal, NULL);
	}
}

static void Configure_GPIO_Interrupts(void) {
	gpio_config_t io_conf = {};
	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	//bit mask of the pin GPIO18
	io_conf.pin_bit_mask = (uint64_t) (1UL << GPIO_NUM_17);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	//install gpio isr service
	gpio_install_isr_service(0);

	// Setting interrupt pin for accel sensor.
	gpio_isr_handler_add(GPIO_NUM_17, gpio_isr_handler, (void*) 17);
}

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

	// todo: create a queue to handle gpio event from isr

    xTaskCreate( systemTask, "System", 2048, nullptr, tskIDLE_PRIORITY , nullptr);
    xTaskCreate( UITask, "UserInterface", 2048, nullptr, tskIDLE_PRIORITY, nullptr);
    xTaskCreate( recorderTask, "Recorder", 2048, nullptr, tskIDLE_PRIORITY, nullptr);

	ESP_LOGI(TAG_Main, "Tasks Created successfully.");
}


