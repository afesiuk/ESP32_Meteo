/*
 * Project name: ESP32_Meteo
 * Author: Alex Fesyuk
 *
 * [BME280]  DRIVER:   https://github.com/BoschSensortec/BME280_driver
 * [MH-Z19B] LIBRARY:  https://github.com/danielealbano/esp32-air-quality-monitor
 * [ESP-IDF] EXAMPLES: https://github.com/espressif/esp-idf
 *
 * UART_NUM_0 - Used for USB-PC transfer [Don't use]
 *
 * BME280 connect (I2C):
 * SDA  - SDA  ESP - [GPIO_NUM_21] (D21)
 * SCL  - SCL  ESP - [GPIO_NUM_22] (D22)
 * 3.3v - 3.3v ESP
 * GND  - GND  ESP
 *
 * MH-Z19B connect (UART):
 * TX   - RX ESP  - [GPIO_NUM_16]  (RX2)
 * RX   - TX ESP  - [GPIO_NUM_17]  (TX2)
 * VIN  - VIN (5v) / 3.3v ESP [Need power from board! | Better to use 5v]
 * GND  - GND ESP
 *
 * HTTP Requests:
 * 1. GET     -  Get data from server (Database)
 * 2. POST    -  Insert data into server (Database)                     *
 * 3. PUT     -  Insert and override data on server (Database)          *
 * 4. DELETE  -  Delete data from server (Database)                     *
 * 5. CONNECT -  Establish a tunnel to the server specified in the URI  *
 * 6. OPTION  -  Get parameters for connecting to the server
 * 7. TRACE   -  Can track what is happening with your requests
 *
 * HTTP Response status codes:
 * 200 - OK
 * 201 - Created
 * 202 - Accepted
 * 204 - No Content
 *
 * 400 - Bad Request
 * 401 - Unauthorized
 * 403 - Forbidden
 * 404 - Not Found
 * 405 - Method Not Allowed
 * 406 - Not Acceptable
 * 408 - Request Timeout
 * 429 - Too Many Requests
 */
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
/* ------ Private Additional includes ------ */
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_netif.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "esp_smartconfig.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"
#include "bme280.h"
#include "mhz19.h"
#include "sensors_support.h"
#include "nvs_flash.h"
/* ------ Private defines ------ */
#define BUFFER_SIZE_T              1024
#define TASK_STACK_SIZE            2048
#define HTTP_TASK_STACK_SIZE       8192
#define MAX_HTTP_RECV_BUFFER       512
#define MAX_HTTP_OUTPUT_BUFFER     2048
#define STRF_TIME_BUFFER           64
#define WIFI_FIELD_SIZE            30

#define TAG_INFO                   "INFO"
#define TAG_NVS                    "NVS Operation"
#define TAG_BME280                 "BME280"
#define TAG_MHZ19B                 "MH-Z19B"
#define TAG_WIFI                   "Wifi_Station"
#define TAG_HTTP                   "HTTP_Client"
#define TAG_TIME                   "Time_SNTP"
#define TAG_SMART_CFG              "Smart_Config"

#define I2C_MASTER_ACK             0
#define I2C_MASTER_NACK            1

#define WIFI_CONNECTED_BIT         BIT0
#define WIFI_FAIL_BIT              BIT1
/* ------ Private functions prototypes ------ */
void init_UART(void);
void init_I2C (void);
void init_GPIO(void);
void init_MH_Z19B(void);
void init_SNTP(void);
void init_Time(void);
void init_Wifi_SmartConfig(void);
esp_err_t init_NVS (void);
int32_t init_BME280(void);

void start_message();
void delay_ms(uint32_t ms);
void Error_Check_MH_Z19B(mhz19_err_t error);
void Message_HTTP_Client(esp_http_client_method_t method);
void time_sync_notification_cb(struct timeval *tv);
void write_wifi_Credentials(uint8_t* ssid, uint8_t* password);
void read_wifi_Credentials(void);

esp_err_t _http_event_handler(esp_http_client_event_t *evt);
static void wait_Connect(EventBits_t uxBits);
static void obtain_time(void);
static void http_request(esp_http_client_method_t method, char* post_data);
static void event_handler(void* arg,
		esp_event_base_t event_base,
		int32_t event_id,
		void* event_data);
/* ------ Private Tasks prototypes ------ */
static void smartconfig_task(void *arg);
static void bme280_check_task(void *arg);
static void mhz19b_check_task(void *arg);
static void http_request_task(void *arg);
static void co2_led_task(void *arg);
/* ------ Private variables ------ */
struct wifi_credentials_t {
	char ssid[WIFI_FIELD_SIZE];
	char password[WIFI_FIELD_SIZE];
}wifi_data;

struct bme280_t bme280;
struct tm timeinfo;

uint8_t xAccessOp    = 0;
uint8_t xSmartConfig = 0;
uint8_t xDefaultMode = 0;

int32_t mhz19b_temperature = 0;
int32_t mhz19b_co2         = 0;

double bme280_temperature = 0;
double bme280_humidity    = 0;
double bme280_pressure    = 0;

char strftime_buf[STRF_TIME_BUFFER] = {0};
char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

static uint8_t retry_cnt = 0;
static EventBits_t uxBits;
static EventGroupHandle_t s_wifi_event_group; // FreeRTOS event group to signal when we are connected
static time_t now;

QueueHandle_t uart_queue;
uart_port_t uart_num = UART_PORT;

/* Requests templates for x-www-form-urlencoded */
const char* bme280_t_req  = "temp_bme";
const char* bme280_h_req  = "hum_bme";
const char* bme280_p_req  = "press_bme";
const char* mhz19b_c_req  = "co2_mhz";
const char* mhz19b_t_req  = "temp_mhz";
const char* date_time_req = "date_time";
/* ------------------- MAIN function ------------------- */
void app_main(void)
{
	/* ---- Init periphal ---- */
	init_GPIO();
	init_I2C();
	init_UART();
	init_NVS();
	init_Wifi_SmartConfig();
	wait_Connect(uxBits); // Wait until device is connected to AP
	init_SNTP();
	init_Time();
	/* --- Start functions --- */
	start_message();
	/* -------- Tasks -------- */
	xTaskCreate(&http_request_task, "http_test_task", TASK_STACK_SIZE * 4, NULL, 5, NULL);
	xTaskCreate(&bme280_check_task, "bme280_check_task", TASK_STACK_SIZE, NULL, 6, NULL);
	xTaskCreate(&mhz19b_check_task, "mhz19b_check_task", TASK_STACK_SIZE, NULL, 7, NULL);
	xTaskCreate(&co2_led_task, "co2_led_task", TASK_STACK_SIZE, NULL, 8, NULL);
}

/* ------------------- Init Periphal ------------------- */
void init_UART(void)
{
	uart_config_t uart_config = {
			.baud_rate = MHZ19B_BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB,
			.rx_flow_ctrl_thresh = 122
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

	i2c_param_config(I2C_PORT, &i2c_config);
	i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

void init_GPIO(void)
{
	gpio_pad_select_gpio(LED_BOARD);
	gpio_set_direction(LED_BOARD, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(RED_LED);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(GREEN_LED);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(BLUE_LED);
	gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);
}

esp_err_t init_NVS(void)
{
	esp_err_t ret = nvs_flash_init();

	if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	return (esp_err_t) ret;
}

void init_Wifi_SmartConfig(void)
{
	xDefaultMode = 1;
	ESP_ERROR_CHECK(esp_netif_init());
	s_wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

	assert(sta_netif);

	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&config));

	esp_event_handler_t instance_any_id;
	esp_event_handler_t instance_got_ip;

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
			ESP_EVENT_ANY_ID, &event_handler, &instance_any_id));

	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
			IP_EVENT_STA_GOT_IP, &event_handler, &instance_got_ip));

	read_wifi_Credentials();

	wifi_config_t wifi_config = {
			.sta = {
					.threshold.authmode = WIFI_AUTH_WPA2_PSK,
					.pmf_cfg = {
							.capable = true,
							.required = false
					},
			},
	};

    /* Set ssid and password fields in wifi_config */
    memcpy(wifi_config.sta.ssid, (uint8_t*) wifi_data.ssid, sizeof(wifi_data.ssid));
    memcpy(wifi_config.sta.password, (uint8_t*) wifi_data.password, sizeof(wifi_data.password));

	ESP_LOGI(TAG_WIFI, "SSID     [config]: %s", wifi_config.sta.ssid);
	ESP_LOGI(TAG_WIFI, "PASSWORD [config]: %s", wifi_config.sta.password);

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

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

		/* Change DefaultMode -> See event_handler */
		xDefaultMode = 0;

		/* Unregister old event handlers */
		ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT,
				ESP_EVENT_ANY_ID, instance_any_id));

		ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT,
				IP_EVENT_STA_GOT_IP, instance_got_ip));

		/* Turn off wifi */
	    ESP_ERROR_CHECK(esp_wifi_disconnect());
	    ESP_ERROR_CHECK(esp_wifi_stop());
	    ESP_ERROR_CHECK(esp_wifi_deinit());

	    /* Reinit s_wifi_event_group */
	    vEventGroupDelete(s_wifi_event_group);
		s_wifi_event_group = xEventGroupCreate();

		/* Init config */
		wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&config));

		/* Register new handlers */
		esp_event_handler_t smartconfig_any_id;
		esp_event_handler_t smartconfig_got_ip;
		ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
				ESP_EVENT_ANY_ID, &event_handler, &smartconfig_any_id));
		ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
				IP_EVENT_STA_GOT_IP, &event_handler, &smartconfig_got_ip));
	    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

	    /* Set mode and start wifi */
	    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	    ESP_ERROR_CHECK(esp_wifi_start());
	}
	else
	{
		ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
	}
}

void init_MH_Z19B(void)
{
	ESP_LOGI(TAG_MHZ19B, "Init MH-Z19B start.");

	mhz19_init(uart_num);
	mhz19_set_auto_calibration(false);
	mhz19_set_range(MEASURE_RANGE);

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

	/* Default settings: pressure - 16X, temperature - 2X, humidity - 1X */
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_PRESSURE);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_TEMP);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_HUMIDITY);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

	ESP_LOGI(TAG_BME280, "Init BME280 done.");

	return (int32_t) com_rslt;
}

void init_SNTP(void)
{
	ESP_LOGI(TAG_TIME, "Init SNTP");
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, URL_NTP_SERVER);
	sntp_set_time_sync_notification_cb(time_sync_notification_cb);
	sntp_init();
}

void init_Time(void)
{
	ESP_LOGI(TAG_TIME, "Time is not set. Setting time over NTP server...");

	obtain_time();
	time(&now);

	/* Set timezone to Europe/Kiev */
	setenv("TZ", FORMAT_TIMEZONE_KIEV, 1);
	tzset();

	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

	ESP_LOGI(TAG_TIME, "Time has been set.");

	ESP_LOGI(TAG_TIME, "Time in Kiev: %s", strftime_buf);
}

/* ------------------- Different functions ------------------- */
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

void Message_HTTP_Client(esp_http_client_method_t method)
{
	switch(method)
	{
		case HTTP_METHOD_GET:
			ESP_LOGI(TAG_HTTP, "HTTP Request: GET");
			break;

		case HTTP_METHOD_POST:
			ESP_LOGI(TAG_HTTP, "HTTP Request: POST");
			break;

		case HTTP_METHOD_PUT:
			ESP_LOGI(TAG_HTTP, "HTTP Request: PUT");
			break;

		case HTTP_METHOD_DELETE:
			ESP_LOGI(TAG_HTTP, "HTTP Request: DELETE");
			break;

		case HTTP_METHOD_HEAD:
			ESP_LOGI(TAG_HTTP, "HTTP Request: HEAD");
			break;

		default:
			ESP_LOGE(TAG_HTTP, "HTTP Request: OTHER [CHECK FOR AVAILABLE METHODS]");
			break;
	}
}

static void wait_Connect(EventBits_t uxBits)
{
	gpio_set_level(LED_BOARD, 1);
	ESP_LOGW(TAG_SMART_CFG, "Waiting for connection to AP.");
	while(!xAccessOp) vTaskDelay(10);
	gpio_set_level(LED_BOARD, 0);
}

void write_wifi_Credentials(uint8_t* ssid, uint8_t* password)
{
	/* Open NVS handle OP */
	ESP_LOGI(TAG_NVS, "Opening NVS handle...");

	nvs_handle_t nvs_handle;

	esp_err_t err = init_NVS();
	err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

	if(err != ESP_OK)
	{
		ESP_LOGI(TAG_NVS, "Error [%s] opening NVS handle.\n", esp_err_to_name(err));
		return;
	}

	/* Write OP */
	ESP_LOGI(TAG_NVS, "Updating SSID and password in NVS...");

	ESP_LOGE(TAG_NVS, "SSID: %s", (char*) ssid);
	ESP_LOGE(TAG_NVS, "PASSWORD: %s", (char*) password);

	err = nvs_set_str(nvs_handle, "wifi_ssid", (char*) ssid);
	ESP_LOGI(TAG_NVS, "%s", err == ESP_OK ? "Done" : "Failed");

	err = nvs_set_str(nvs_handle, "wifi_password", (char*) password);
	ESP_LOGI(TAG_NVS, "%s", err == ESP_OK ? "Done" : "Failed");

	/* Commit value in NVS OP */
	ESP_LOGI(TAG_NVS, "Committing updates in NVS...");
	err = nvs_commit(nvs_handle);
	ESP_LOGI(TAG_NVS, "%s", err == ESP_OK ? "Done" : "Failed");

	/* Close NVS */
	nvs_close(nvs_handle);
}

void read_wifi_Credentials(void)
{
	/* Open NVS handle OP */
	ESP_LOGI(TAG_NVS, "Opening NVS handle...");

	nvs_handle_t nvs_handle;

	esp_err_t err = init_NVS();
	err = nvs_open("storage", NVS_READONLY, &nvs_handle);

	if(err != ESP_OK)
	{
		ESP_LOGI(TAG_NVS, "Error [%s] opening NVS handle.\n", esp_err_to_name(err));

		ESP_LOGI(TAG_NVS, "Set default values to wifi_data.");
	    memcpy(wifi_data.ssid, (uint8_t*) "default", 7);
	    memcpy(wifi_data.password, (uint8_t*) "default", 7);
	}
	else
	{
		/* Read SSID OP */
		ESP_LOGI(TAG_NVS, "Done.");

		ESP_LOGI(TAG_NVS, "Reading SSID from NVS...");

		size_t ssid_len = sizeof(wifi_data.ssid);
		err = nvs_get_str(nvs_handle, "wifi_ssid", wifi_data.ssid, &ssid_len);

		switch(err)
		{
		case ESP_OK:
			ESP_LOGI(TAG_NVS, "Done.");
			break;

		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG_NVS, "The value is not init yet.");
			break;

		default:
			ESP_LOGI(TAG_NVS, "Error [%s] reading.", esp_err_to_name(err));
			break;
		}

		/* Read PASSWORD OP */
		ESP_LOGI(TAG_NVS, "Reading PASSWORD from NVS...");

		size_t pass_len = sizeof(wifi_data.password);
		err = nvs_get_str(nvs_handle, "wifi_password", wifi_data.password, &pass_len);

		switch(err)
		{
		case ESP_OK:
			ESP_LOGI(TAG_NVS, "Done.");
			ESP_LOGI(TAG_NVS, "NVS Wifi_SSID: %s", wifi_data.ssid);
			ESP_LOGI(TAG_NVS, "NVS Wifi_Password: %s", wifi_data.password);
			break;

		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG_NVS, "The value is not init yet.");
			break;

		default:
			ESP_LOGI(TAG_NVS, "Error [%s] reading.", esp_err_to_name(err));
			break;
		}
	}

	nvs_close(nvs_handle);
}
/* ------------------- WI-FI functions / handlers ------------------- */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		if(xDefaultMode)
		{
			esp_wifi_connect();
		}
		else
		{
			xTaskCreate(smartconfig_task, "smartconfig_task", TASK_STACK_SIZE * 2, NULL, 3, NULL);
		}

	}
	else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if(xDefaultMode)
		{
			if(retry_cnt < ESP_MAXIMUM_RETRY)
			{
				esp_wifi_connect();
				retry_cnt++;
				ESP_LOGI(TAG_WIFI, "Retry to connect to the AP.");
			}
			else
			{
				xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
			}
			ESP_LOGI(TAG_WIFI, "Connect to the AP fail.");
		}
		else
		{
			esp_wifi_connect();
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		}
	}
	else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		if(xDefaultMode)
		{
			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
			ESP_LOGI(TAG_WIFI, "Got ip: " IPSTR, IP2STR(&event->ip_info.ip));
			retry_cnt = 0;
		}
		ESP_LOGI(TAG_WIFI, "IP_EVENT_STA_GOT_IP");
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
		xAccessOp = 1;
	}
	else if(event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
	{
		ESP_LOGI(TAG_SMART_CFG, "Scan done.");
	}
	else if(event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
	{
		ESP_LOGI(TAG_SMART_CFG, "Found channel");
	}
	else if(event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
	{
		ESP_LOGI(TAG_SMART_CFG, "Got SSID and Password.");

		uint8_t ssid[33] = {0};
		uint8_t password[65] = {0};

		smartconfig_event_got_ssid_pswd_t *event = (smartconfig_event_got_ssid_pswd_t*) event_data;
		wifi_config_t wifi_config;

		bzero(&wifi_config, sizeof(wifi_config_t));

		memcpy(wifi_config.sta.ssid, event->ssid, sizeof(wifi_config.sta.ssid));
		memcpy(wifi_config.sta.password, event->password, sizeof(wifi_config.sta.password));

		wifi_config.sta.bssid_set = event->bssid_set;

		if(wifi_config.sta.bssid_set == true)
		{
			memcpy(wifi_config.sta.bssid, event->bssid, sizeof(wifi_config.sta.bssid));
		}

		memcpy(ssid, event->ssid, sizeof(event->ssid));
		memcpy(password, event->password, sizeof(event->password));

		ESP_LOGI(TAG_SMART_CFG, "SSID: %s", ssid);
		ESP_LOGI(TAG_SMART_CFG, "Password: %s", password);

		/* Set fields of wifi to NVS */
		write_wifi_Credentials(ssid, password);

		ESP_ERROR_CHECK(esp_wifi_disconnect());
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
		ESP_ERROR_CHECK(esp_wifi_connect());
	}
	else if(event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
	{
		xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
	}
}
/* ------------------- HTTP functions / handlers ------------------- */
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
	static char* output_buffer;
	static int output_len;

	switch(evt->event_id)
	{
		case HTTP_EVENT_ERROR:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ERROR");
			break;

		case HTTP_EVENT_ON_CONNECTED:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_CONNECTED");
			break;

		case HTTP_EVENT_HEADER_SENT:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_HEADER_SENT");
			break;

		case HTTP_EVENT_ON_HEADER:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_HEADER, key=%s, value=%s",
					evt->header_key, evt->header_value);
			break;

		case HTTP_EVENT_ON_DATA:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
			if(!esp_http_client_is_chunked_response(evt->client))
			{
				if(evt->user_data)
				{
					memcpy(evt->user_data + output_len, evt->data, evt->data_len);
				}
				else
				{
					if(output_buffer == NULL)
					{
						output_buffer = (char*) malloc (esp_http_client_get_content_length(evt->client));
						output_len = 0;
						if(output_buffer == NULL)
						{
							ESP_LOGE(TAG_HTTP, "Failed to allocate memory for output buffer");
							return ESP_FAIL;
						}
					}
					memcpy(output_buffer + output_len, evt->data, evt->data_len);
				}
				output_len += evt->data_len;
			}
			break;

		case HTTP_EVENT_ON_FINISH:
			ESP_LOGD(TAG_HTTP, "HTTP_EVENT_ON_FINISH");
			if(output_buffer != NULL)
			{
				ESP_LOG_BUFFER_HEX(TAG_HTTP, output_buffer, output_len);
				free(output_buffer);
				output_buffer = NULL;
			}
			output_len = 0;
			break;

		case HTTP_EVENT_DISCONNECTED:
			ESP_LOGI(TAG_HTTP, "HTTP_EVENT_DISCONNECTED");
			int mbedtls_err = 0;
			esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
			if(err != 0)
			{
				if(output_buffer != NULL)
				{
					free(output_buffer);
					output_buffer = NULL;
				}
				output_len = 0;
				ESP_LOGI(TAG_HTTP, "Last ESP Error code: 0x%x", err);
				ESP_LOGI(TAG_HTTP, "Last MbedTLS failure: 0x%x", mbedtls_err);
			}
			break;
	}

	return ESP_OK;
}

static void http_request(esp_http_client_method_t method, char* post_data)
{
	esp_http_client_config_t http_config = {
			.host = IP_ADDR_SERVER,
			.port = PORT_SERVER,
			.path = PATH_SERVER,
			.method = method,
			.event_handler = _http_event_handler,
			.user_data = local_response_buffer
	};

	esp_http_client_handle_t client = esp_http_client_init(&http_config);

	/* If method PUT or POST -> we need to reconfigure http header */
	if(method == HTTP_METHOD_POST || method == HTTP_METHOD_PUT)
	{
		ESP_LOGI(TAG_HTTP, "Body of request: %s", post_data);
		esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
		esp_http_client_set_post_field(client, post_data, strlen(post_data));
	}

	/* Start HTTP request */
	esp_err_t err = esp_http_client_perform(client);

	Message_HTTP_Client(method);

	if(err == ESP_OK)
	{
		ESP_LOGI(TAG_HTTP, "HTTP Status = %d, content_length = %d",
				esp_http_client_get_status_code(client),
				esp_http_client_get_content_length(client));
	}
	else
	{
		ESP_LOGE(TAG_HTTP, "HTTP Request failed: %s",
				esp_err_to_name(err));
	}

	/* If method GET or HEAD -> check local response buffer */
	if(method == HTTP_METHOD_GET || method == HTTP_METHOD_HEAD)
	{
		ESP_LOG_BUFFER_HEX(TAG_HTTP, local_response_buffer, strlen(local_response_buffer));
	}

	esp_http_client_cleanup(client);
}
/* ------------------- SNTP functions / handlers ------------------- */
void time_sync_notification_cb(struct timeval *tv)
{
	ESP_LOGI(TAG_TIME, "Notification of a time synchronization event");
}

static void obtain_time(void)
{
	struct tm timeinfo = {0};
	time_t now = 0;
	uint8_t retry = 0;
	uint8_t retry_count = SNTP_RETRY_CONNECT;

	while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
	{
		ESP_LOGI(TAG_TIME, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		delay_ms(2000);
	}

	time(&now);
	localtime_r(&now, &timeinfo);
}

static void update_time(void)
{
	time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

	ESP_LOGI(TAG_TIME, "Time has been updated.");
}
/* ------------------- TASKS ------------------- */
static void smartconfig_task(void *arg)
{
	ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));

	smartconfig_start_config_t config = SMARTCONFIG_START_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_smartconfig_start(&config));

	for(;;)
	{
		uxBits = xEventGroupWaitBits(s_wifi_event_group,
				WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, true, false, portMAX_DELAY);

		if(uxBits & WIFI_CONNECTED_BIT)
		{
			ESP_LOGI(TAG_SMART_CFG, "Device connected to AP.");
			xAccessOp = 1;
		}
		if(uxBits & WIFI_FAIL_BIT)
		{
			ESP_LOGI(TAG_SMART_CFG, "SmartConfig task is over.");
			esp_smartconfig_stop();
			vTaskDelete(NULL);
		}
	}
}

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
			delay_ms(5000);
			continue;
		}

		com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

		if(com_rslt == SUCCESS)
		{
			bme280_temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
			bme280_pressure    = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa
			bme280_humidity    = bme280_compensate_humidity_double(v_uncomp_humidity_s32);

			ESP_LOGI(TAG_BME280, "%.2f C | %.0f hPa | %.2f %%",
					bme280_temperature, bme280_pressure, bme280_humidity);
		}
		else
		{
			/* data of sensor -> 0 'cause we don't get actual data */
			bme280_temperature = 0;
			bme280_pressure = 0;
			bme280_humidity = 0;

			ESP_LOGE(TAG_BME280, "Measure error | code: %d", com_rslt);
		}

		delay_ms(BME280_MEASURE_DELAY);
	}

	vTaskDelete(NULL);
}

static void mhz19b_check_task(void *arg)
{
	init_MH_Z19B();

	mhz19_err_t mhz19b_error;

	delay_ms(START_MEASURE_DELAY);

	for(;;)
	{
		delay_ms(MHZ19B_MEASURE_DELAY);

		mhz19b_error = mhz19_retrieve_data();

		if(mhz19b_error != MHZ19_ERR_OK)
		{
			Error_Check_MH_Z19B(mhz19b_error);
			/* data of sensor -> 0 'cause we don't get actual data */
			mhz19b_co2 = 0;
			mhz19b_temperature = 0;
			continue;
		}

		mhz19b_co2  = mhz19_get_co2() * 0.4;
		mhz19b_temperature = mhz19_get_temperature();

		ESP_LOGI(TAG_MHZ19B, "CO2: %d | Temperature: %d", mhz19b_co2, mhz19b_temperature);
	}

	vTaskDelete(NULL);
}

static void co2_led_task(void *arg)
{
	/* Wait until MH-Z19B will get actual data */
	delay_ms(START_MEASURE_DELAY);

	for(;;)
	{
		/* If the MH-Z19B doesn't work properly -> turn off rgb-led */
		if(mhz19b_co2 == 0)
		{
			gpio_set_level(RED_LED, 0);
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(BLUE_LED, 0);
		}
		else if(mhz19b_co2 <= 800)
		{
			/* Set green color */
			gpio_set_level(RED_LED, 0);
			gpio_set_level(GREEN_LED, 1);
			gpio_set_level(BLUE_LED, 0);
		}
		else if(mhz19b_co2 > 800 && mhz19b_co2 <= 1400)
		{
			/* Set yellow color */
			gpio_set_level(RED_LED, 1);
			gpio_set_level(GREEN_LED, 1);
			gpio_set_level(BLUE_LED, 0);
		}
		else
		{
			/* Set red color */
			gpio_set_level(RED_LED, 1);
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(BLUE_LED, 0);
		}

		delay_ms(2000);
	}

	vTaskDelete(NULL);
}

static void http_request_task(void *arg)
{
	char* request = NULL;

	/* TODO: [After test] Set delay for > 10 min 'cause MH-Z19B need more time to get correct data */
	delay_ms(START_HTTP_DELAY);

	for(;;)
	{
		update_time();
		/* Create body of request to server */
		asprintf(&request, "%s=%ld&%s=%ld&%s=%.2f&%s=%.2f&%s=%.2f&%s=%s",
				mhz19b_c_req, (long int) mhz19b_co2,
				mhz19b_t_req, (long int) mhz19b_temperature,
				bme280_t_req, bme280_temperature,
				bme280_h_req, bme280_humidity,
				bme280_p_req, bme280_pressure,
				date_time_req, strftime_buf);
		/* Create and send HTTP request */
		http_request(HTTP_METHOD_POST, (char*) request);
		/* free allocated memory */
		free(request);

		delay_ms(REQUEST_HTTP_DELAY);
	}

	vTaskDelete(NULL);
}
