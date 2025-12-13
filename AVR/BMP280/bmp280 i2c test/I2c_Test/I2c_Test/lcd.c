#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "lcd.h" // Assuming this header file exists with function prototypes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>

// Pin mapping based on user hardware connections:
// rs (Arduino D7) -> PD7
// en (Arduino D6) -> PD6
// d4 (Arduino D5) -> PD5
// d5 (Arduino D4) -> PD4
// d6 (Arduino D3) -> PD3
// d7 (Arduino D2) -> PD2

/* Port definitions - all pins are now on PORTD for simplicity */
#define LCD_PORT    PORTD
#define LCD_DDR     DDRD

/* Pin definitions on PORTD */
#define LCD_RS_PIN PD7
#define LCD_EN_PIN PD6
// Data pins D4..D7 map to PD5..PD2
#define LCD_D4_PIN PD5
#define LCD_D5_PIN PD4
#define LCD_D6_PIN PD3
#define LCD_D7_PIN PD2
// Backlight pin (BL_PIN) is undefined in the new hardware mapping, 
// so backlight control functions have been commented out.

/* Low-level helpers */
static void pulse_enable(void)
{
	LCD_PORT |= (1 << LCD_EN_PIN);
	_delay_us(1);
	LCD_PORT &= ~(1 << LCD_EN_PIN);
	_delay_us(100);
}

static void write4(uint8_t nibble)
{
	// Map the 4-bit nibble to the new specific physical data pins (PD5 down to PD2)
	if (nibble & 0x01) LCD_PORT |=  (1 << LCD_D4_PIN); else LCD_PORT &= ~(1 << LCD_D4_PIN);
	if (nibble & 0x02) LCD_PORT |=  (1 << LCD_D5_PIN); else LCD_PORT &= ~(1 << LCD_D5_PIN);
	if (nibble & 0x04) LCD_PORT |=  (1 << LCD_D6_PIN); else LCD_PORT &= ~(1 << LCD_D6_PIN);
	if (nibble & 0x08) LCD_PORT |=  (1 << LCD_D7_PIN); else LCD_PORT &= ~(1 << LCD_D7_PIN);
	pulse_enable();
}

static void lcd_cmd_raw(uint8_t cmd)
{
	// RS = 0 (Command mode)
	LCD_PORT &= ~(1 << LCD_RS_PIN);
	_delay_us(1);
	write4((cmd >> 4) & 0x0F);
	write4(cmd & 0x0F);
	_delay_ms(2);
}

static void lcd_data_raw(uint8_t data)
{
	// RS = 1 (Data mode)
	LCD_PORT |= (1 << LCD_RS_PIN);
	_delay_us(1);
	write4((data >> 4) & 0x0F);
	write4(data & 0x0F);
	_delay_us(50);
}

/* Public API */
void lcd_init(void)
{
	// Configure all used pins (D7, D6, D5, D4, D3, D2) as outputs
	LCD_DDR |= (1 << LCD_RS_PIN) | (1 << LCD_EN_PIN) | 
	           (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) | 
	           (1 << LCD_D6_PIN) | (1 << LCD_D7_PIN);

	// Ensure outputs are low initially
	LCD_PORT &= ~((1 << LCD_RS_PIN) | (1 << LCD_EN_PIN) |
	              (1 << LCD_D4_PIN) | (1 << LCD_D5_PIN) | 
	              (1 << LCD_D6_PIN) | (1 << LCD_D7_PIN));

	// Small startup delay
	_delay_ms(50);

	// HD44780 4-bit init sequence
	// send 0x03 three times
	LCD_PORT &= ~(1 << LCD_RS_PIN);
	write4(0x03);
	_delay_ms(5);
	write4(0x03);
	_delay_us(200);
	write4(0x03);
	_delay_us(200);
	// set 4-bit mode
	write4(0x02);
	_delay_us(200);

	// function set: 4-bit, 2 lines, 5x8
	lcd_cmd_raw(0x28);
	// display on, cursor off, blink off
	lcd_cmd_raw(0x0C);
	// entry mode
	lcd_cmd_raw(0x06);
	// clear display
	lcd_cmd_raw(0x01);
	_delay_ms(2);
	
}

void lcd_clear(void)
{
	lcd_cmd_raw(0x01);
	_delay_ms(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
	uint8_t addr = (row == 0) ? 0x00 : 0x40;
	addr += col;
	lcd_cmd_raw(0x80 | addr);
}

void lcd_puts_at(uint8_t row, uint8_t col, const char *s)
{
	char tmp[17];
	strncpy(tmp, s, 16);
	tmp[16] = '\0';

	lcd_set_cursor(row, col);
	const char *p = tmp;
	while (*p) {
		lcd_data_raw((uint8_t)*p++);
	}
	// pad remaining positions with spaces to clear previous text
	uint8_t len = strlen(tmp);
	for (uint8_t i = len; i < 16; ++i) lcd_data_raw(' ');
}