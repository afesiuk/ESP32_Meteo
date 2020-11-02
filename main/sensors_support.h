#ifndef MAIN_SENSORS_SUPPORT_H_
#define MAIN_SENSORS_SUPPORT_H_

#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "bme280.h"

int8_t BME280_I2C_Write(uint8_t dev_addr,
		uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);

int8_t BME280_I2C_Read(uint8_t dev_addr,
		uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);


#endif /* MAIN_SENSORS_SUPPORT_H_ */
