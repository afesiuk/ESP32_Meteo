#include "mhz19.h"

static const char* TAG = "MHZ19";

static uint32_t millis()
{
    return (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
}

void mhz19_init(uart_port_t uart_num)
{
	MHZ19_data_t._mhz19_result = MHZ19_ERR_NO_DATA;
	MHZ19_data_t._mhz19_cmd = 0;
	MHZ19_data_t._uart_num = uart_num;
}

int mhz19_bytes2int(uint8_t h, uint8_t l)
{
    return (256 * ((int)h)) + ((int)l);
}

void mhz19_print_buffer(uint8_t is_request, unsigned char* buffer)
{
#if MHZ19_PRINT_BUFFER == 1
    char message[60];
    memset(message, 0, sizeof(message));

    sprintf(message, "%s ", is_request ? "Request >>" : "Response <<");
    for(uint8_t i = 0; i < 9; i++)
    {
        sprintf((message + strlen(message)), "0x%02x ", buffer[i]);
    }

    ESP_LOGI(TAG, "%s", message);
#endif
}

uint8_t mhz19_calc_crc(uint8_t data[])
{
    uint8_t i;
    uint8_t crc = 0;
    for (i = 1; i < 8; i++)
    {
        crc += data[i];
    }
    crc = 255 - crc;
    crc++;

    return crc;
}

void mhz19_prepare_for_write_discard_data_in_uart()
{
    size_t length;
	uint8_t buf[20];

	do
	{
		ESP_ERROR_CHECK(uart_get_buffered_data_len(MHZ19_data_t._uart_num, (size_t*)&length));
		if (length > 0)
		{
			uart_read_bytes(MHZ19_data_t._uart_num, buf, length > sizeof(buf) ? sizeof(buf) : length, 100);
		}

		taskYIELD();
	}
	while(length > 0);

    return;
}

size_t mhz19_write(uint8_t data[], uint8_t len)
{
    size_t length;

    mhz19_prepare_for_write_discard_data_in_uart();
	length = uart_write_bytes(MHZ19_data_t._uart_num, (const char*)data, 9);

 	ESP_ERROR_CHECK(uart_flush(MHZ19_data_t._uart_num));

    return length;
}

void mhz19_send_command(uint8_t command, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
	MHZ19_data_t._mhz19_cmd = command;

    uint8_t cmd[9] = { 0xFF, 0x01, command, b3, b4, b5, b6, b7, 0x00 };
    cmd[8] = mhz19_calc_crc(cmd);

    mhz19_print_buffer(1, cmd);
    mhz19_write(cmd, 9);  
}

size_t mhz19_read(uint8_t data[], uint8_t len)
{
    size_t length;
	unsigned long start_millis = millis();

	memset(data, 0, len);

	do
	{
		ESP_ERROR_CHECK(uart_get_buffered_data_len(MHZ19_data_t._uart_num, (size_t*)&length));

		if (millis() - start_millis >= MHZ19_REQUEST_TIMEOUT_PERIOD) 
		{
			ESP_LOGW(TAG, "Timed out waiting for response");    
			return MHZ19_ERR_TIMEOUT;
		}

		taskYIELD();
	}
	while(length <= 0);
    
	length = uart_read_bytes(MHZ19_data_t._uart_num, data, len, 100);
    mhz19_print_buffer(0, data);

    return length;
}

mhz19_err_t mhz19_receive_response(uint8_t cmd[9])
{
    mhz19_err_t err = (mhz19_err_t)mhz19_read(cmd, 9);

    if (err < MHZ19_ERR_OK)
    {
    	MHZ19_data_t._mhz19_result = err;
    }
    else if (cmd[0] != 0xFF)
    {
    	MHZ19_data_t._mhz19_result = MHZ19_ERR_INVALID_RESPONSE;
    }
    else if (cmd[1] != MHZ19_data_t._mhz19_cmd)
    {
    	MHZ19_data_t._mhz19_result = MHZ19_ERR_UNEXPECTED_CMD;
    }
    else if (cmd[8] != mhz19_calc_crc(cmd))
    {
    	MHZ19_data_t._mhz19_result = MHZ19_ERR_WRONG_CRC;
    }
    else
    {
    	MHZ19_data_t._mhz19_result = MHZ19_ERR_OK;
    }
    
    return MHZ19_data_t._mhz19_result;
}

mhz19_err_t mhz19_retrieve_data()
{
    mhz19_send_command(0x86, 0, 0, 0, 0, 0);
    return mhz19_receive_response(MHZ19_data_t._mhz19_response);
}

void mhz19_set_auto_calibration(bool mode)
{
    uint8_t value = mode ? 0xA0 : 0x00;
    mhz19_send_command(0x79, value, 0x00, 0x00, 0x00, 0x00);
}

void mhz19_set_range(int range)
{
    if (range != 2000 && range != 5000)
    {
        return;
    }

    uint8_t range_high = (range >> 8) & 0xFF;
    uint8_t range_low = range & 0xFF;

    mhz19_send_command(0x99, range_high, range_low, 0x00, 0x00, 0x00);
}

int mhz19_get_co2()
{
    if (MHZ19_data_t._mhz19_result == MHZ19_ERR_OK)
    {
        return mhz19_bytes2int(MHZ19_data_t._mhz19_response[2],
        		MHZ19_data_t._mhz19_response[3]);
    }

    return MHZ19_data_t._mhz19_result;
}

int mhz19_get_temperature()
{
    if (MHZ19_data_t._mhz19_result == MHZ19_ERR_OK)
    {
        int value = (int)MHZ19_data_t._mhz19_response[4];
        return value - 40;
    }

    return MHZ19_data_t._mhz19_result;
}
