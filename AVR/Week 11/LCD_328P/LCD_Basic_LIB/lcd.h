#ifndef LCD_H_
#define LCD_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// LCD pin definitions based on standard Keypad Shield wiring for ATmega328P
#define LCD_DATA_PORT PORTD
#define LCD_DATA_DDR DDRD
#define LCD_RS_PORT PORTB
#define LCD_RS_DDR DDRB
#define LCD_E_PORT PORTB
#define LCD_E_DDR DDRB

// Corresponding ATmega328P pins (Port B) for Arduino digital pins 8 (RS) and 9 (E)
#define LCD_RS_PIN 0 // Arduino D8 is PB0
#define LCD_E_PIN 1  // Arduino D9 is PB1

// Keypad Analog input pin on ATmega328P (PC0)
#define KEYPAD_PIN 0 // A0 on Arduino is PC0

// ADC threshold values (approximate) for the 5 buttons based on the voltage divider circuit
#define KEY_RIGHT_MIN 0
#define KEY_RIGHT_MAX 50
#define KEY_UP_MIN 80
#define KEY_UP_MAX 150
#define KEY_DOWN_MIN 290
#define KEY_DOWN_MAX 350
#define KEY_LEFT_MIN 480
#define KEY_LEFT_MAX 550
#define KEY_SELECT_MIN 650
#define KEY_SELECT_MAX 750
// Enum for button states
typedef enum {
	NO_KEY,
	KEY_RIGHT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_SELECT
} Button;

// LCD function prototypes
void lcd_init(void);
void lcd_send_cmd(unsigned char command);
void lcd_send_char(unsigned char character);
void lcd_send_string(char *string);
void lcd_clear(void);
void lcd_goto_xy(unsigned char x, unsigned char y);

// Keypad function prototype
Button read_keypad(void);
void adc_init(void);

#endif /* LCD_H_ */
