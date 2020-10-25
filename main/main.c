/*
 * Project name: ESP32_Meteo
 * Author: Alex Fesyuk
 *
 * [BME280]  DRIVER:  https://github.com/BoschSensortec/BME280_driver
 * [MH-Z19B] LIBRARY: https://github.com/danielealbano/esp32-air-quality-monitor
 *
 * UART_NUM_0 - Used for USB-PC transfer [Don't use]
 *
 * BME280 connect (I2C):
 * SDA  - SDA  ESP - [GPIO_NUM_21] (D21)
 * SCL  - SCL  ESP - [GPIO_NUM_22] (D22)
 * 3.3v - 3.3v ESP
 * GND  - GND  ESP
 *
 * MH-Z19B connect (UART_NUM_2):
 * TX   - RX ESP  - [GPIO_NUM_16]  (RX2)
 * RX   - TX ESP  - [GPIO_NUM_17]  (TX2)
 * VIN  - VIN (5v) / 3.3v ESP [Need power from board! | Better to use 5v]
 * GND  - GND ESP
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
/* ------ Private Additional includes ------ */
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "bme280.h"
#include "mhz19.h"
#include "nvs_flash.h"

/* ------ Private defines ------ */
#define UART_TX_PIN                GPIO_NUM_17
#define UART_RX_PIN                GPIO_NUM_16

#define MHZ19B_BAUDRATE            9600
#define BUFFER_SIZE_T              1024
#define TASK_STACK_SIZE            2048

#define TAG_INFO                   "INFO"
#define TAG_BME280                 "BME280"
#define TAG_MHZ19B                 "MH-Z19B"
#define TAG_WIFI                   "Wifi_Station"

#define LED_BOARD                  GPIO_NUM_2
#define SDA_PIN                    GPIO_NUM_21
#define SCL_PIN                    GPIO_NUM_22

#define I2C_MASTER_ACK             0
#define I2C_MASTER_NACK            1
#define ESP_MAXIMUM_RETRY          5

#define WIFI_SSID                  "AirPort Extreme"
#define WIFI_PASS                  "6735s41wty801"

#define WIFI_CONNECTED_BIT         BIT0
#define WIFI_FAIL_BIT              BIT1

/* ------ Private functions prototypes ------ */
void init_UART(void);
void init_I2C (void);
void init_GPIO(void);
void init_NVS (void);
void init_WIFI_Station(void);
void init_MH_Z19B(void);

void start_message();
void delay_ms(uint32_t ms);
void Error_Check_MH_Z19B(mhz19_err_t error);

int32_t init_BME280(void);

static void event_handler(void* arg,
		esp_event_base_t event_base,
		int32_t event_id,
		void* event_data);

int8_t BME280_I2C_Write(uint8_t dev_addr,
		uint8_t reg_addr,
		uint8_t* reg_data,
		uint8_t count);

int8_t BME280_I2C_Read(uint8_t dev_addr,
		uint8_t reg_addr,
		uint8_t* reg_data,
		uint8_t count);

/* ------ Private Tasks prototypes ------ */
static void bme280_check_task(void *arg);
static void mhz19b_check_task(void *arg);

/* ------ Private variables ------ */
struct bme280_t bme280;

static uint8_t retry_try = 0;

int32_t mhz19b_temperature = 0;
int32_t mhz19b_co2 = 0;
int32_t range = 2000;

double bme280_temperature = 0;
double bme280_huminidy = 0;
double bme280_pressure = 0;

static EventGroupHandle_t s_wifi_event_group; // FreeRTOS event group to signal when we are connected

QueueHandle_t uart_queue;

uart_port_t uart_num = UART_NUM_2;
/* ------------------- MAIN function ------------------- */
void app_main(void)
{
	/* ---- Init periphal ---- */
    init_GPIO();
    init_I2C();
    init_UART();
    init_NVS();
	init_WIFI_Station();
	/* ---- Start functions before scheduler ---- */
	start_message();
	/* ---- Tasks ---- */
	xTaskCreate(&bme280_check_task, "bme280_check_task", TASK_STACK_SIZE, NULL, 6, NULL);
	xTaskCreate(&mhz19b_check_task, "mhz19b_check_task", TASK_STACK_SIZE, NULL, 25, NULL);
}

/* ------------------- Init Periphal ------------------- */
void init_UART(void)
{
	/* Init UART for MH-Z19B */
	uart_config_t uart_config = {
			.baud_rate = MHZ19B_BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
			.rx_flow_ctrl_thresh = 122 // 122
	};

	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

	ESP_ERROR_CHECK(uart_set_pin(uart_num,
				UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	ESP_ERROR_CHECK(uart_driver_install(uart_num,
			BUFFER_SIZE_T * 2, BUFFER_SIZE_T * 2, 100, &uart_queue, 0));
}

void init_I2C(void)
{
	i2c_config_t i2c_config = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = SDA_PIN,
			.scl_io_num = SCL_PIN,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 100000
	};

	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void init_GPIO(void)
{
	gpio_pad_select_gpio(LED_BOARD);
	gpio_set_direction(LED_BOARD, GPIO_MODE_OUTPUT);
	//gpio_set_level(LED_BOARD, 0);
}

void init_NVS(void)
{
	esp_err_t ret = nvs_flash_init();

	if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);
}

void init_WIFI_Station(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&config));

	esp_event_handler_t instance_any_id;
	esp_event_handler_t instance_got_ip;

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
			ESP_EVENT_ANY_ID, &event_handler, &instance_any_id));

	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
			IP_EVENT_STA_GOT_IP, &event_handler, &instance_got_ip));

    wifi_config_t wifi_config = {
    		.sta = {
    				.ssid = WIFI_SSID,
					.password = WIFI_PASS,
					.threshold.authmode = WIFI_AUTH_WPA2_PSK,
					.pmf_cfg = {
							.capable = true,
							.required = false
    				},
    		},
    };

	ESP_LOGI(TAG_WIFI, "SSID [config]: %s", wifi_config.sta.ssid);
	ESP_LOGI(TAG_WIFI, "PASSWORD [config]: %s", wifi_config.sta.password);

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG_WIFI, "Wifi-init finished.");

	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
			WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdFALSE,
			pdFALSE,
			portMAX_DELAY);

	if(bits & WIFI_CONNECTED_BIT)
	{
		ESP_LOGI(TAG_WIFI, "Connected to Wi-fi Access Point, SSID: %s | Password: %s",
				wifi_config.sta.ssid, wifi_config.sta.password);
	}
	else if(bits & WIFI_FAIL_BIT)
	{
		ESP_LOGI(TAG_WIFI, "Failed to connect, SSID: %s | Password: %s",
				wifi_config.sta.ssid, wifi_config.sta.password);
	}
	else
	{
		ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
	}

	ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT,
			IP_EVENT_STA_GOT_IP, instance_got_ip));

	ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT,
				ESP_EVENT_ANY_ID, instance_any_id));

	vEventGroupDelete(s_wifi_event_group);
}

void init_MH_Z19B(void)
{
	ESP_LOGI(TAG_MHZ19B, "Init MH-Z19B start.");

	mhz19_init(uart_num);
	mhz19_set_auto_calibration(false);
	mhz19_set_range(range);

	ESP_LOGI(TAG_MHZ19B, "Init MH-Z19B done.");
}

int32_t init_BME280(void)
{
	ESP_LOGI(TAG_BME280, "Init BME280 start.");

	/* Init BME280 struct */
	bme280.bus_write  = BME280_I2C_Write;
	bme280.bus_read   = BME280_I2C_Read;
	bme280.dev_addr   = BME280_I2C_ADDRESS1;
	bme280.delay_msec = delay_ms;

	int32_t com_rslt = bme280_init(&bme280);

	/* Set 16x oversampling for more correct data */
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_16X);

	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_OFF);

	ESP_LOGI(TAG_BME280, "Init BME280 done.");

	return (int32_t) com_rslt;
}


/* ------------------- Other functions ------------------- */
void delay_ms(uint32_t ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

void start_message()
{
    /* Print chip information */
    esp_chip_info_t chip_info;

    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_INFO, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG_INFO, "Silicon revision: %d, ", chip_info.revision);

    ESP_LOGI(TAG_INFO, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG_INFO, "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());
}

void Error_Check_MH_Z19B(mhz19_err_t error)
{
	switch(error)
	{
		case MHZ19_ERR_NO_DATA:
			ESP_LOGE(TAG_MHZ19B, "MH-Z19B Error: no data");
			break;

		case MHZ19_ERR_TIMEOUT:
			ESP_LOGE(TAG_MHZ19B, "MH-Z19B Error: timeout");
			break;

		case MHZ19_ERR_INVALID_RESPONSE:
			ESP_LOGE(TAG_MHZ19B, "MH-Z19B Error: invalid response");
			break;

		case MHZ19_ERR_UNEXPECTED_CMD:
			ESP_LOGE(TAG_MHZ19B, "MH-Z19B Error: unexpected command");
			break;

		case MHZ19_ERR_WRONG_CRC:
			ESP_LOGE(TAG_MHZ19B, "MH-Z19B Error: wrong checksum");
			break;

		case MHZ19_ERR_OK:
			break;
	}
}

/* ------------------- BME280 functions ------------------- */
int8_t BME280_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt)
{
	int32_t iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

	if(espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (int8_t) iError;
}

int8_t BME280_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt)
{
	int32_t iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if(cnt > 1) i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);

	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

	if(espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (int8_t) iError;
}
/* ------------------- WI-FI functions / handlers ------------------- */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		esp_wifi_connect();
	}
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if(retry_try < ESP_MAXIMUM_RETRY)
		{
			esp_wifi_connect();
			retry_try++;
			ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
		}
		else
		{
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG_WIFI, "Connect to the AP fail");
	}
	else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG_WIFI, "Got ip: " IPSTR, IP2STR(&event->ip_info.ip));
		retry_try = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}
/* ------------------- TASKS ------------------- */
static void bme280_check_task(void *arg)
{
	int32_t com_rslt = init_BME280();

	int32_t v_uncomp_pressure_s32;
	int32_t v_uncomp_temperature_s32;
	int32_t v_uncomp_humidity_s32;

	for(;;)
	{
		if(com_rslt != SUCCESS)
		{
			ESP_LOGE(TAG_BME280, "Init or setting error | Code: %d", com_rslt);
			com_rslt = init_BME280();
			vTaskDelay(5000 / portTICK_PERIOD_MS);
			continue;
		}

		com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

		if(com_rslt == SUCCESS)
		{
			bme280_temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
			bme280_pressure    = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
			bme280_huminidy    = bme280_compensate_humidity_double(v_uncomp_humidity_s32);

			ESP_LOGI(TAG_BME280, "%.2f C | %.0f hPa | %.2f %%",
					bme280_temperature, bme280_pressure, bme280_huminidy);
		}
		else
		{
			ESP_LOGE(TAG_BME280, "Measure error | code: %d", com_rslt);
		}

		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

static void mhz19b_check_task(void *arg)
{
	init_MH_Z19B();

	mhz19_err_t mhz19b_error;

	for(;;)
	{
		vTaskDelay(10000 / portTICK_PERIOD_MS);

		mhz19b_error = mhz19_retrieve_data();

		if(mhz19b_error != MHZ19_ERR_OK)
		{
			Error_Check_MH_Z19B(mhz19b_error);
			continue;
		}

		mhz19b_co2  = mhz19_get_co2();
		mhz19b_temperature = mhz19_get_temperature();

		ESP_LOGI(TAG_MHZ19B, "CO2: %d | Temperature: %d", mhz19b_co2, mhz19b_temperature);
	}

	vTaskDelete(NULL);
}
