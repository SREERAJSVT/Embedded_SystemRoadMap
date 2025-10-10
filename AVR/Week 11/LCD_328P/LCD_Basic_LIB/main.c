#include "lcd.h"

int main(void) {
	// Initialize peripherals
	adc_init();
	lcd_init();

	lcd_send_string("Hi *o*--*o*");
	lcd_goto_xy(1, 2);
	lcd_send_string("!");

	while (1) {
		Button key = read_keypad();

		// Display the pressed key
		lcd_goto_xy(1, 2);
		switch (key) {
			case KEY_RIGHT:
			lcd_send_string("Right       ");
			break;
			case KEY_UP:
			lcd_send_string("Up          ");
			break;
			case KEY_DOWN:
			lcd_send_string("Down        ");
			break;
			case KEY_LEFT:
			lcd_send_string("Left        ");
			break;
			case KEY_SELECT:
			lcd_send_string("Select      ");
			break;
			default:
			lcd_send_string("No Key      ");
			break;
		}
	}
}
