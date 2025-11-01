#define F_CPU 16000000UL // For a 16 MHz clock, like on the Arduino Uno

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "UART_H.h" // Your custom UART header

// Define the LED pin as Arduino D7
#define LED_PORT    PORTD
#define LED_DDR     DDRD
#define LED_PIN     PD7

// Define the Chip Select (CS) pin for SPI as Arduino D10
#define CS_PORT     PORTB
#define CS_DDR      DDRB
#define CS_PIN      PB2

// BMP280 register addresses
#define BMP280_REG_CHIPID       0xD0
#define BMP280_CHIP_ID_VAL      0x58
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB_00     0x88

// BMP280 control values
#define BMP280_MODE_NORMAL      0x03
#define BMP280_OVERSAMP_16X     0x05
#define BMP280_STANDBY_1000MS   0x05

// Global variables to store calibration parameters
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

// Function prototypes for LED, SPI, and BMP280
void spi_init(void);
uint8_t spi_transfer(uint8_t data);
uint8_t bmp280_read_register(uint8_t reg_addr);
void bmp280_read_registers(uint8_t reg_addr, uint8_t *data, uint8_t len);
void setup_led(void);
void turn_on_led(void);
void turn_off_led(void);
uint8_t check_bmp280_connection(void);
void bmp280_init(void);
void bmp280_read_calibration_data(void);
int32_t bmp280_compensate_T_int32(int32_t adc_T);
int32_t bmp280_compensate_P_int64(int32_t adc_P);
void bmp280_write_register(uint8_t reg_addr, uint8_t data);

// --- Main Program ---
int main(void) {
	long temperature_scaled, pressure_scaled;
	
	setup_led();
	turn_off_led();
	UART_Init();

	UART_TxString("Starting SPI connection check...\r\n");

	spi_init();

	if (check_bmp280_connection()) {
		UART_TxString("Success: BMP280 connection established.\r\n");
		turn_on_led();
		bmp280_init();
		} else {
		UART_TxString("Error: BMP280 not found or connection failed.\r\n");
		turn_off_led();
		while (1);
	}
	
	while(1) {
		uint8_t data[6];
		bmp280_read_registers(BMP280_REG_PRESS_MSB, data, 6);
		
		int32_t raw_pressure = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
		int32_t raw_temperature = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

		temperature_scaled = bmp280_compensate_T_int32(raw_temperature);
		pressure_scaled = bmp280_compensate_P_int64(raw_pressure);

		UART_TxString("--Temp: ");
		UART_TxNumber(temperature_scaled / 100);
		UART_TxChar('.');
		UART_TxNumber(temperature_scaled % 100);
		UART_TxString(" C  -- A Pressure: ");
		UART_TxNumber(pressure_scaled / 100);
		UART_TxChar('.');
		UART_TxNumber(pressure_scaled % 100);
		UART_TxString("hpa \r\n");
		
		_delay_ms(2000);
	}
	
	return 0;
}


// --- SPI and BMP280 Functions ---
void spi_init(void) {
	DDRB |= (1 << PB3) | (1 << PB5) | (1 << CS_PIN);
	DDRB &= ~(1 << PB4);
	CS_PORT |= (1 << CS_PIN);
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t spi_transfer(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

uint8_t bmp280_read_register(uint8_t reg_addr) {
	uint8_t data;
	CS_PORT &= ~(1 << CS_PIN);
	spi_transfer(reg_addr | 0x80);
	data = spi_transfer(0x00);
	CS_PORT |= (1 << CS_PIN);
	return data;
}

void bmp280_read_registers(uint8_t reg_addr, uint8_t *data, uint8_t len) {
	CS_PORT &= ~(1 << CS_PIN);
	spi_transfer(reg_addr | 0x80);
	for (uint8_t i = 0; i < len; i++) {
		data[i] = spi_transfer(0x00);
	}
	CS_PORT |= (1 << CS_PIN);
}

uint8_t check_bmp280_connection(void) {
	uint8_t chip_id = bmp280_read_register(BMP280_REG_CHIPID);
	if (chip_id == BMP280_CHIP_ID_VAL) {
		return 1;
	}
	return 0;
}

void bmp280_init(void) {
	_delay_ms(100);
	
	bmp280_read_calibration_data();

	bmp280_write_register(BMP280_REG_CONFIG, (BMP280_STANDBY_1000MS << 5));
	bmp280_write_register(BMP280_REG_CTRL_MEAS, (BMP280_OVERSAMP_16X << 5) | (BMP280_OVERSAMP_16X << 2) | BMP280_MODE_NORMAL);
}

void bmp280_read_calibration_data(void) {
	uint8_t cal_data[24];
	bmp280_read_registers(BMP280_REG_CALIB_00, cal_data, 24);

	dig_T1 = (uint16_t)cal_data[1] << 8 | (uint16_t)cal_data[0];
	dig_T2 = (int16_t)cal_data[3] << 8 | (int16_t)cal_data[2];
	dig_T3 = (int16_t)cal_data[5] << 8 | (int16_t)cal_data[4];

	dig_P1 = (uint16_t)cal_data[7] << 8 | (uint16_t)cal_data[6];
	dig_P2 = (int16_t)cal_data[9] << 8 | (int16_t)cal_data[8];
	dig_P3 = (int16_t)cal_data[11] << 8 | (int16_t)cal_data[10];
	dig_P4 = (int16_t)cal_data[13] << 8 | (int16_t)cal_data[12];
	dig_P5 = (int16_t)cal_data[15] << 8 | (int16_t)cal_data[14];
	dig_P6 = (int16_t)cal_data[17] << 8 | (int16_t)cal_data[16];
	dig_P7 = (int16_t)cal_data[19] << 8 | (int16_t)cal_data[18];
	dig_P8 = (int16_t)cal_data[21] << 8 | (int16_t)cal_data[20];
	dig_P9 = (int16_t)cal_data[23] << 8 | (int16_t)cal_data[22];
}

void bmp280_write_register(uint8_t reg_addr, uint8_t data) {
	CS_PORT &= ~(1 << CS_PIN);
	spi_transfer(reg_addr & 0x7F);
	spi_transfer(data);
	CS_PORT |= (1 << CS_PIN);
}

int32_t bmp280_compensate_T_int32(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

int32_t bmp280_compensate_P_int64(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return p / 256;
}

// --- LED Functions ---
void setup_led(void) {
	LED_DDR |= (1 << LED_PIN);
}

void turn_on_led(void) {
	LED_PORT |= (1 << LED_PIN);
}

void turn_off_led(void) {
	LED_PORT &= ~(1 << LED_PIN);
}
/*
// --- UART Functions ---
void UART_Init(void) {
	UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
	UBRR0L = (unsigned char)UBRR_VALUE;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void UART_TxChar(char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}
void UART_TxString(const char *str) {
	while (*str) {
		UART_TxChar(*str++);
	}
}
*/