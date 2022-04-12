/* File: recorder.cpp
 * Author: Nick Gkloumpos
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <recorder.hh>
#include <esp_log.h>
#include "SensorInterface/sensorInterface.h"



extern "C" void recorderTask(void * parameter) {
	ESP_LOGI(TAG_Recorder, "Recording task initiated");

	/* todo: Create a sensor class to aggregate all the sensors in one place
	 * And simplify the recording and initialization procedure.
	 */

	/* Read sensor conversion registers for test! */
//	uint8_t totalRes = I2C_STATUS_SUCCESS;
//	i2c_status_t currentRes = I2C_STATUS_SUCCESS;
//	while(totalRes == I2C_STATUS_SUCCESS)
//	{
//		ESP_LOGW(TAG_SensorInterface,"Acc_X: %d ", sensor.GetAccel_X_Raw(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Acc_Y: %d ", sensor.GetAccel_Y_Raw(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Acc_Z: %d ", sensor.GetAccel_Z_Raw(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Temp: %.2f ", sensor.GetTemperature_Celcius(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Gyro_X: %d ", sensor.GetGyro_X_Raw(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Gyro_Y: %d ", sensor.GetGyro_Y_Raw(&currentRes));
//		totalRes |= currentRes;
//		ESP_LOGW(TAG_SensorInterface,"Gyro_Z: %d \n", sensor.GetGyro_Z_Raw(&currentRes));
//		totalRes |= currentRes;
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//	}
}
