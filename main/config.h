#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

/* ----------------------- GPIO ---------------------- */
#define RED_LED                    GPIO_NUM_25
#define GREEN_LED                  GPIO_NUM_26
#define BLUE_LED                   GPIO_NUM_27
#define LED_BOARD                  GPIO_NUM_2
/* ---------------------- Wi-Fi ---------------------- */
#define WIFI_SSID                  "AirPort Extreme" // -> Get from NVS
#define WIFI_PASS                  "6735s41wty801"   // -> Get from NVS
#define ESP_MAXIMUM_RETRY          5
/* ---------------------- HTTP ----------------------- */
#define START_HTTP_DELAY           60000
#define REQUEST_HTTP_DELAY         60000
/* --------------------- I2C Pins -------------------- */
#define I2C_PORT                   I2C_NUM_0
#define SDA_PIN                    GPIO_NUM_21
#define SCL_PIN                    GPIO_NUM_22
/* -------------------- UART Pins -------------------- */
#define UART_PORT                  UART_NUM_2
#define UART_TX_PIN                GPIO_NUM_17
#define UART_RX_PIN                GPIO_NUM_16
/* ---------------- Server with MongoDB -------------- */
#define IP_ADDR_SERVER             "192.168.1.192"
#define PATH_SERVER                "/sensors"
#define PORT_SERVER                8000  // Default for tcp transport_t: (80 or 443)
/* --------------- SNTP server / time ---------------- */
#define URL_NTP_SERVER             "pool.ntp.org"
#define FORMAT_TIMEZONE_KIEV       "EET-2EEST,M3.5.0/3,M10.5.0/4"
/* ----------------- MH-Z19B settings ---------------- */
#define MHZ19B_BAUDRATE            9600
#define MEASURE_RANGE              2000
#define START_MEASURE_DELAY        30000
#define MHZ19B_MEASURE_DELAY       10000
/* ----------------- BME280 settings ----------------- */
#define BME280_MEASURE_DELAY       10000

#endif /* MAIN_CONFIG_H_ */
