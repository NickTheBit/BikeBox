//
// Created by nick on 09/04/2022.
//

#include <mpu6050/driverInterface.h>
#include "mpu6050/i2c_interface.h"

/**
* @brief  I2C peripheral initialization method.
* @param  clock I2C clock frequency (default 100 kHz)
* @retval i2c_status_t
*/
i2c_status_t ESP32_I2C_IF::Init_I2C(i2c_clockspeed_t clock) {
	master_conf.master.clk_speed = (uint32_t) clock;
	i2c_param_config(I2C_Master_Port, &master_conf);
	esp_err_t status = i2c_driver_install(I2C_Master_Port, I2C_MODE_MASTER, 16, 16, 0);
	if (status != ESP_OK) {
		return I2C_STATUS_ERROR;
	}
	return I2C_STATUS_SUCCESS;
}

/**
* @brief  This method will be used for reading the data of the given register from
* the slave with given address.
* @param  slaveAddress Slave chip I2C bus address
* @param  regAddress Register address to be read
* @param  status Pointer for operation status
* @retval uint8_t Read register value
*/
uint8_t ESP32_I2C_IF::ReadRegister(uint8_t slaveAddress, uint8_t regAddress, i2c_status_t *status) {
	uint8_t readBuffer;
	esp_err_t err = i2c_master_read_from_device(I2C_Master_Port, slaveAddress, &readBuffer, sizeof(uint8_t), I2C_Timeout_ms);

	switch (err) {
		case ESP_OK:
			*status = I2C_STATUS_SUCCESS;
			break;
		case ESP_FAIL:
			*status = I2C_STATUS_ERROR;
			break;
		default:
			*status = I2C_STATUS_NONE;
			break;
	}
	return readBuffer;
}

/**
* @brief  This method will be used for writing gven data to the given register of the slave device
* with the given address.
* @param  slaveAddress Slave chip I2C bus address
* @param  regAddress Register address that the data to be written
* @param  data Data to be written
* @retval i2c_status_t
*/
i2c_status_t ESP32_I2C_IF::WriteRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data) {
	esp_err_t err = i2c_master_write_to_device(I2C_Master_Port, slaveAddress, &data, sizeof(uint8_t), I2C_Timeout_ms);
	if (err != ESP_OK) {
		return I2C_STATUS_ERROR;
	}
	return I2C_STATUS_SUCCESS;
}

/**
* @brief  Class destructor.
* @param  none
* @retval none
*/
ESP32_I2C_IF::~ESP32_I2C_IF() {
	i2c_driver_delete(I2C_Master_Port);
}
