#define F_CPU 16000000UL // For a 16 MHz /*week 13 live class 2303/010112025
//this code change initialized global pointers to identify.
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "UART_H.h" // Your custom UART header

// Define the LED pin as Arduino D7
#define LED_PORT    PORTD // output pins 
#define LED_DDR     DDRD //
#define LED_PIN     PD7 //pin high

// Define the Chip Select (CS) 10 pb2  th pin pin for SPI as Arduino D10
#define CS_PORT     PORTB //chip selection  
#define CS_DDR      DDRB
#define CS_PIN      PB2

// BMP280 register addresses
#define BMP280_REG_CHIPID   0xD0 // 0x00 hex
#define BMP280_CHIP_ID_VAL  0x58

// Function prototypes for LED, SPI, and BMP280
void spi_init(void);
uint8_t spi_transfer(uint8_t data);
uint8_t bmp280_read_register(uint8_t reg_addr);
void setup_led(void);
void turn_on_led(void);
void turn_off_led(void);
uint8_t check_bmp280_connection(void);

int main(void) {
	// 1. booting animation  Set up the LED and UART for output
	setup_led();
	turn_on_led();
	_delay_ms(500);
	turn_off_led();
	_delay_ms(300);
	turn_on_led();
	_delay_ms(500);
	turn_off_led();
	_delay_ms(500);
	turn_off_led();
	turn_on_led();
	_delay_ms(500);
	turn_off_led();
	_delay_ms(300);
	turn_on_led();
	_delay_ms(500);
	turn_off_led();
	_delay_ms(500);
	turn_off_led();
	//end ani
	UART_Init();

	UART_TxString("Starting SPI connection check...\r\n");

	// 2. Initialize SPI
	spi_init();

	// 3. Check for the BMP280 sensor connection
	if (check_bmp280_connection()) {
		// If connection is successful, print a message and turn on the LED
		UART_TxString("Success: BMP280 connection established.\r\n");
		turn_on_led();		
		} else {
		// If connection fails, print an error and halt
		UART_TxString("Error: BMP280 not found or connection failed.\r\n");
		turn_off_led();
		while (1);
	}
	// The program will stay here after a successful check
	while(1);
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
uint8_t check_bmp280_connection(void) {
	uint8_t chip_id = bmp280_read_register(BMP280_REG_CHIPID);
	if (chip_id == BMP280_CHIP_ID_VAL) {
		return 1;
	}
	return 0;
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
// used putty and verifyed com 4 out.