#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

/* Credentials */
#define WIFI_SSID                  "AirPort Extreme"
#define WIFI_PASS                  "6735s41wty801"
/* I2C Pins */
#define SDA_PIN                    GPIO_NUM_21
#define SCL_PIN                    GPIO_NUM_22
/* UART Pins */
#define UART_TX_PIN                GPIO_NUM_17
#define UART_RX_PIN                GPIO_NUM_16
/* Server with MongoDB */
#define URL_POST                   "http://192.168.1.192:8000/sensors"
/* SNTP server */
#define URL_SNTP                   "pool.ntp.org"

#endif /* MAIN_CONFIG_H_ */
