/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"

#include <recorder.hh>
#include <esp_log.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

#include <mpu6050/driverInterface.h>
#include <mpu6050.h>

// Queue for sensor read events.
static xQueueHandle qSensorReadEvent = NULL;

static const gpio_num_t MPUInterruptPin = GPIO_NUM_18;

extern "C" void recorderTask(void * parameter) {
	ESP_LOGI(TAG_Recorder, "Recording task initiated");

	/* create i2c and sensor class instances and configure the sensor */
	I2C_Interface *i2c_if = new ESP32_I2C_IF();
	/* initialize fast I2C 400 kHz */
	if (i2c_if->Init_I2C(i2c_clockspeed_t::CLK_400KHz) != I2C_STATUS_SUCCESS) {
		printf("I2C initialization failed!\n");
		delete i2c_if;
		esp_restart();
	}

	MPU6050_Driver::MPU6050 sensor(i2c_if);
	if(sensor.ResetSensor() != I2C_STATUS_SUCCESS)
	{
		printf("Sensor reset failed!\n");
		esp_restart();
	}
	/* Simple safety delay after sensor reset */
	vTaskDelay(100 / portTICK_RATE_MS);

	/* Wakeup sensor and set full scale ranges */
	if(sensor.InitializeSensor(MPU6050_Driver::Gyro_FS_t::FS_1000_DPS, MPU6050_Driver::Accel_FS_t::FS_8G) != I2C_STATUS_SUCCESS)
	{
		printf("Sensor initialization failed!\n");
		esp_restart();
	}

	/* Auto-Calibrate gyroscope registers to target value 0 (default) */
	if(sensor.Calibrate_Gyro_Registers() != I2C_STATUS_SUCCESS)
	{
		printf("Gyro calibration failed!\n");
		esp_restart();
	}

	/* Auto-Calibrate accelerometer registers to target values by default:
     * X = 0 MG, Y = 0 MG, Z = 1 MG */
	if(sensor.Calibrate_Accel_Registers() != I2C_STATUS_SUCCESS)
	{
		printf("Accel calibration failed!\n");
		esp_restart();
	}

	/* set digital low pass to BW_184Hz (sample rate reduced to 1KHz)
     * (just to show the feature it already has default value in startup) */
	if(sensor.SetSensor_DLPF_Config(MPU6050_Driver::DLPF_t::BW_184Hz) != I2C_STATUS_SUCCESS) {
		printf("DLPF configuration failed!\n");
		esp_restart();
	}

	/* set sample rate divider for 20 Hz sample rate.
     * 1000 / (1 + 49) = 20 Hz */
	if(sensor.SetGyro_SampleRateDivider(49) != I2C_STATUS_SUCCESS) {
		printf("Sample rate config failed!\n");
		esp_restart();
	}

	/* set sensor interrput coinfig to default value! */
	if(sensor.SetSensor_InterruptPinConfig(0x00) != I2C_STATUS_SUCCESS) {
		printf("Interrupt pin configuration failed!\n");
		esp_restart();
	}

	/* Enable sensor data ready interrupt. */
	uint8_t interruptConfigVal = MPU6050_Driver::Regbits_INT_ENABLE::BIT_DATA_RDY_EN;
	if(sensor.SetSensor_InterruptEnable(interruptConfigVal) != I2C_STATUS_SUCCESS) {
		printf("Interrput enable failed!\n");
		esp_restart();
	}

	/* Make sure sensor fifo is disabledbefore reseting the buffer. */
	if(sensor.SetSensor_FIFO_Enable(false) != I2C_STATUS_SUCCESS) {
		printf("Sensor fifo enable failed!\n");
		esp_restart();
	}

	/* Reset sensro fifo buffer. */
	if(sensor.Reset_Sensor_FIFO() != I2C_STATUS_SUCCESS) {
		printf("Sensor fifo reset failed!\n");
		esp_restart();
	}

	/* Enable sensor fifo. */
	if(sensor.SetSensor_FIFO_Enable(true) != I2C_STATUS_SUCCESS) {
		printf("Sensor fifo enable failed!\n");
		esp_restart();
	}

	/* configure sensor fifo to be able to read accelerometer x, y, z and
     * gyroscope x, y ,z values. */
	uint8_t fifoConfigVal =
			MPU6050_Driver::Regbits_FIFO_EN::BIT_ACCEL_FIFO_EN |
			MPU6050_Driver::Regbits_FIFO_EN::BIT_XG_FIFO_EN |
			MPU6050_Driver::Regbits_FIFO_EN::BIT_YG_FIFO_EN |
			MPU6050_Driver::Regbits_FIFO_EN::BIT_ZG_FIFO_EN;
	if(sensor.SetSensor_FIFO_Config(fifoConfigVal) != I2C_STATUS_SUCCESS) {
		printf("Sensor fifo congfiguration failed!\n");
		esp_restart();
	}

	i2c_status_t error = I2C_STATUS_NONE;
	float currentSampleRateHz = sensor.GetSensor_CurrentSampleRate_Hz(&error);
	if(error != I2C_STATUS_SUCCESS) {
		printf("Sample rate reading failed!\n");
		esp_restart();
	}

	printf("Sensor sample rate: %.2f Hz \n", currentSampleRateHz);
	printf("Sensor configuration completed!\n");

	uint8_t intStatus = 0;
	uint16_t fifoCount = 0;
	uint8_t fifoData[12]; // axH, axL, ayH, ayL, azH, azL, gxH, gxL, gyH, gyL, gzH, gzL
	SensorFrame currentFrame; // structure to keep current frame in constructed int16_t form
	constexpr uint16_t FIFO_FRAME_LEN = 12;

	uint8_t qData;
	while (error == I2C_STATUS_SUCCESS)
	{
		/* read sensor FIFO buffer on every data ready interrupt */
		if (xQueueReceive(qSensorReadEvent, &qData, portMAX_DELAY) == pdPASS)
		{
			intStatus = sensor.GetSensor_InterruptStatus(&error);

			if(error != I2C_STATUS_SUCCESS) {
				printf("Interrupt status read failed!\n");
				break;
			}

			/* Check if there is any overflow. If yes then abort! */
			if(intStatus & MPU6050_Driver::Regbits_INT_ENABLE::BIT_FIFO_OFLOW_EN) {
				printf("FIFO overflow detected!\n");
				break;
			}

			fifoCount = sensor.GetSensor_FIFOCount(&error);

			if(error != I2C_STATUS_SUCCESS) {
				printf("FIFO count read failed!\n");
				break;
			}

			/* If sensor fifo count is smaller than our frame size or if the data ready bit is not set,
             * do not execute the further sequence where we read and process FIFO data. */
			if(fifoCount < FIFO_FRAME_LEN || !(intStatus & MPU6050_Driver::Regbits_INT_ENABLE::BIT_DATA_RDY_EN)) {
				continue;
			}

			/* Read sensor frame amount of bytes (12 bytes) from FIFO buffer. */
			for(uint8_t i = 0; i < FIFO_FRAME_LEN && (error == I2C_STATUS_SUCCESS); i++) {
				fifoData[i] = sensor.GetSensor_FIFO_Data(&error);
			}

			if(error != I2C_STATUS_SUCCESS) {
				printf("FIFO data read failed!\n");
				break;
			}

			/* Set sensor frame structure values from latest sensor FIFO data. */
			currentFrame.ax = (int16_t)fifoData[0] << 8 | (int16_t)fifoData[1];
			currentFrame.ay = (int16_t)fifoData[2] << 8 | (int16_t)fifoData[3];
			currentFrame.az = (int16_t)fifoData[4] << 8 | (int16_t)fifoData[5];
			currentFrame.gx = (int16_t)fifoData[6] << 8 | (int16_t)fifoData[7];
			currentFrame.gy = (int16_t)fifoData[8] << 8 | (int16_t)fifoData[9];
			currentFrame.gz = (int16_t)fifoData[10] << 8 | (int16_t)fifoData[11];

			/* print the constructed sensor FIFO data! */
			printf("aX: %d aY: %d aZ: %d gX: %d gY: %d gZ: %d\n",
				   currentFrame.ax,
				   currentFrame.ay,
				   currentFrame.az,
				   currentFrame.gx,
				   currentFrame.gy,
				   currentFrame.gz);
		}
	}

	esp_restart();

}

static void Configure_GPIO_Interrupt(void)
{
	gpio_config_t io_conf = {};
	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	//bit mask of the pin GPIO18
	io_conf.pin_bit_mask = (uint64_t) (1UL << MPUInterruptPin);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	//install gpio isr service
	gpio_install_isr_service(0);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(MPUInterruptPin, gpio_isr_handler, (void*) MPUInterruptPin);
}


/**
 * @brief Interrupt service routine for external pin interrupt (GPIO_NUM_18). Sensor is
 * configured to generate raising edge interrupt on every data ready event. Sensor INT pin is
 * connected to the GPIO_NUM18 pin. Every time we get positive edge interrupt we send sensor
 * read event message to sensor read task via qSensorReadEvent queue.
 * @param arg void argpointer to be casted to desired parameter
 * @retval none
 */
void IRAM_ATTR gpio_isr_handler(uint32_t arg)
{
	uint32_t gpio_num = arg;
	if(gpio_num == MPUInterruptPin) {
		uint8_t qSignal = 1; // dummy signal value for the queue
		xQueueSendFromISR(qSensorReadEvent, &qSignal, NULL);
	}
}

