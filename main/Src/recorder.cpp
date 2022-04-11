/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <recorder.hh>
#include <esp_log.h>

#include <mpu6050/mpu6050.h>
#include <mpu6050/i2c_interface.h>
#include <mpu6050/driverInterface.h>

extern "C" void recorderTask(void * parameter) {
	ESP_LOGI(TAG_Recorder, "Recording task initiated");

	// Initializing i2cInterface interface and sensors
	ESP32_I2C_IF *i2cInterface = new ESP32_I2C_IF();
	if (i2cInterface->Init_I2C(i2c_clockspeed_t::CLK_400KHz) != I2C_STATUS_SUCCESS) {
		ESP_LOGW(TAG_Recorder,"I2C initialization failed!\n");
		delete i2cInterface;
		esp_restart();
	}

	MPU6050_Driver::MPU6050 sensor(i2cInterface);
	if(sensor.ResetSensor() != I2C_STATUS_SUCCESS) {
		ESP_LOGW(TAG_Recorder,"Sensor reset failed!\n");
		esp_restart();
	}
	ESP_LOGI(TAG_Recorder,"Sensor reset completed!\n");
	/* Simple safety delay after sensor reset */
	vTaskDelay(100);

	/* Wakeup sensor and set full scale ranges */
	if(sensor.InitializeSensor(MPU6050_Driver::Gyro_FS_t::FS_1000_DPS, MPU6050_Driver::Accel_FS_t::FS_8G) != I2C_STATUS_SUCCESS)
	{
		ESP_LOGW(TAG_Recorder,"Sensor initialization failed!\n");
		esp_restart();
	}
	ESP_LOGI(TAG_Recorder,"Sensor initialization completed!\n");

	/* Auto-Calibrate gyroscope registers to target value 0 (default) */
	if(sensor.Calibrate_Gyro_Registers() != I2C_STATUS_SUCCESS)
	{
		ESP_LOGW(TAG_Recorder,"Gyro calibration failed!\n");
		esp_restart();
	}

	/* Auto-Calibrate accelerometer registers to target values by default:
	 * X = 0 MG, Y = 0 MG, Z = 1 MG */
	if(sensor.Calibrate_Accel_Registers() != I2C_STATUS_SUCCESS)
	{
		ESP_LOGW(TAG_Recorder,"Accel calibration failed!\n");
		esp_restart();
	}

	/* set digital low pass to default value
	 * (just to show the feature it already has default value in startup) */
	if(sensor.SetSensor_DLPF_Config(MPU6050_Driver::DLPF_t::BW_260Hz) != I2C_STATUS_SUCCESS) {
		ESP_LOGW(TAG_Recorder,"DLPF configuration failed!\n");
		esp_restart();
	}

	/* set sample rate divider to default value
	 * (just to show the feature it already has default value in startup) */
	i2c_status_t error = I2C_STATUS_NONE;
	if(sensor.SetGyro_SampleRateDivider(0) != I2C_STATUS_SUCCESS) {
		ESP_LOGW(TAG_Recorder,"Sample rate config failed!\n");
		esp_restart();
	}
	ESP_LOGI(TAG_Recorder,"Sample rate reading Succeeded!\n");

	float currentSampleRateHz = sensor.GetSensor_CurrentSampleRate_Hz(&error);
	if(error != I2C_STATUS_SUCCESS) {
		ESP_LOGW(TAG_Recorder,"Sample rate reading failed!\n");
		esp_restart();
	}
	ESP_LOGI(TAG_Recorder,"Sample rate reading Succeeded!\n");

	ESP_LOGW(TAG_Recorder,"Sensor sample rate: %.2f Hz \n", currentSampleRateHz);
	ESP_LOGW(TAG_Recorder,"Sensor configuration completed!\n");

	/* Read sensor conversion registers for test! */
	uint8_t totalRes = I2C_STATUS_SUCCESS;
	i2c_status_t currentRes = I2C_STATUS_SUCCESS;
	while(totalRes == I2C_STATUS_SUCCESS)
	{
		ESP_LOGW(TAG_Recorder,"Acc_X: %d ", sensor.GetAccel_X_Raw(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Acc_Y: %d ", sensor.GetAccel_Y_Raw(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Acc_Z: %d ", sensor.GetAccel_Z_Raw(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Temp: %.2f ", sensor.GetTemperature_Celcius(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Gyro_X: %d ", sensor.GetGyro_X_Raw(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Gyro_Y: %d ", sensor.GetGyro_Y_Raw(&currentRes));
		totalRes |= currentRes;
		ESP_LOGW(TAG_Recorder,"Gyro_Z: %d \n", sensor.GetGyro_Z_Raw(&currentRes));
		totalRes |= currentRes;
		vTaskDelay(50 );
	}

	ESP_LOGW(TAG_Recorder,"Sensor read error! Program terminated!\n");
	esp_restart();
}

