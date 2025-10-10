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

void adc_init(void) {
	// Enable ADC, set prescaler to 128 (for 16MHz clock -> 125kHz ADC clock)
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// Set reference voltage to AVCC
	ADMUX |= (1 << REFS0);
	// Select ADC channel 0 (PC0 for A0)
	ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
}

Button read_keypad(void) {
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	unsigned int adc_value = ADC;

	// Check for each button within its analog range
	if (adc_value > KEY_RIGHT_MIN && adc_value < KEY_RIGHT_MAX) {
		return KEY_RIGHT;
		} else if (adc_value > KEY_UP_MIN && adc_value < KEY_UP_MAX) {
		return KEY_UP;
		} else if (adc_value > KEY_DOWN_MIN && adc_value < KEY_DOWN_MAX) {
		return KEY_DOWN;
		} else if (adc_value > KEY_LEFT_MIN && adc_value < KEY_LEFT_MAX) {
		return KEY_LEFT;
		} else if (adc_value > KEY_SELECT_MIN && adc_value < KEY_SELECT_MAX) {
		return KEY_SELECT;
		} else {
		return NO_KEY;
	}
}
