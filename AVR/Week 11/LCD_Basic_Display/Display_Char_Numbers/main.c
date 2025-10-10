#include "lcd.h"

int main(void) {
	// Initialize the LCD display
	lcd_init();

	lcd_goto_xy(1, 1);
	lcd_send_string("Hi Lcd");
	_delay_ms(3000);
	lcd_clear();
	int counter = 0;
	float pi_val = 3.14159;

	while (1) {
		// Display the changing integer on the second line
		lcd_goto_xy(1, 2);
		lcd_send_string("Counter: ");
		
		lcd_send_int(counter);
		lcd_goto_xy(1, 1);
		lcd_send_string("Float: ");
		lcd_send_float(pi_val, 4); // Display float with 4 decimal places
		
		counter++;
		pi_val += 0.01;

		_delay_ms(1000); // Wait for 1 second
		lcd_clear(); // Clear the display for the next message
	}
}
/*//_delay_ms(100); // Wait for 0.1 second

//lcd_clear(); // Clear the display for the next message

// Display the float on the first line
lcd_goto_xy(8, 1);
//lcd_send_string("Float: ");*/