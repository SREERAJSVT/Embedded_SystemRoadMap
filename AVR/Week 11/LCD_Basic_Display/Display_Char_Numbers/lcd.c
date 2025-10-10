#include "lcd.h"

void lcd_init(void) {
	// Set data pins (PD4-PD7), RS (PB0), and E (PB1) as outputs
	LCD_DATA_DDR |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
	LCD_RS_DDR |= (1 << LCD_RS_PIN);
	LCD_E_DDR |= (1 << LCD_E_PIN);

	_delay_ms(20);

	// Manual initialization sequence for 4-bit mode
	lcd_send_cmd(0x33);
	lcd_send_cmd(0x32);
	lcd_send_cmd(0x28); // 4-bit mode, 2 lines, 5x8 font
	lcd_send_cmd(0x0C); // Display on, cursor off
	lcd_send_cmd(0x06); // Increment cursor
	lcd_send_cmd(0x01); // Clear display
	_delay_ms(2);
}

void lcd_send_cmd(unsigned char command) {
	// Send high nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (command & 0xF0);
	LCD_RS_PORT &= ~(1 << LCD_RS_PIN); // Command mode (RS=0)
	LCD_E_PORT |= (1 << LCD_E_PIN);    // Enable pulse
	_delay_us(1);
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	_delay_us(200);

	// Send low nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (command << 4);
	LCD_E_PORT |= (1 << LCD_E_PIN);    // Enable pulse
	_delay_us(1);
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	_delay_ms(2);
}

void lcd_send_char(unsigned char character) {
	// Send high nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (character & 0xF0);
	LCD_RS_PORT |= (1 << LCD_RS_PIN);  // Data mode (RS=1)
	LCD_E_PORT |= (1 << LCD_E_PIN);    // Enable pulse
	_delay_us(1);
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	_delay_us(200);

	// Send low nibble
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (character << 4);
	LCD_E_PORT |= (1 << LCD_E_PIN);    // Enable pulse
	_delay_us(1);
	LCD_E_PORT &= ~(1 << LCD_E_PIN);
	_delay_ms(2);
}

void lcd_send_string(char *string) {
	while (*string) {
		lcd_send_char(*string++);
	}
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
	_delay_ms(2);
}

void lcd_goto_xy(unsigned char x, unsigned char y) {
	unsigned char first_char_addr[] = {0x80, 0xC0};
	lcd_send_cmd(first_char_addr[y-1] + x-1);
	_delay_us(100);
}

// Function to display integers
void lcd_send_int(int number) {
	char buffer[10]; // Buffer to hold the converted string
	itoa(number, buffer, 10); // Convert integer to a base-10 string
	lcd_send_string(buffer);
}

// Function to display floats
void lcd_send_float(float number, unsigned char precision) {
	char buffer[10]; // Buffer to hold the converted string
	// dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
	dtostrf(number, 0, precision, buffer);
	lcd_send_string(buffer);
}
