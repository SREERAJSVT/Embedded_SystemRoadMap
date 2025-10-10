#ifndef LCD_H_
#define LCD_H_

#ifndef F_CPU
#define F_CPU 16000000UL // Set CPU frequency to 16MHz
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h> // For itoa() and dtostrf()

/* --- Pin Definitions for DFRobot LCD Shield --- */
#define LCD_DATA_PORT PORTD
#define LCD_DATA_DDR DDRD
#define LCD_RS_PORT PORTB
#define LCD_RS_DDR DDRB
#define LCD_E_PORT PORTB
#define LCD_E_DDR DDRB

// Corresponding ATmega328P pins for Arduino D8 (RS) and D9 (E)
#define LCD_RS_PIN 0 // Arduino D8 is PB0
#define LCD_E_PIN 1  // Arduino D9 is PB1

/* --- Function Prototypes --- */
void lcd_init(void);
void lcd_send_cmd(unsigned char command);
void lcd_send_char(unsigned char character);
void lcd_send_string(char *string);
void lcd_clear(void);
void lcd_goto_xy(unsigned char x, unsigned char y);
void lcd_send_int(int number);
void lcd_send_float(float number, unsigned char precision);

#endif /* LCD_H_ */
