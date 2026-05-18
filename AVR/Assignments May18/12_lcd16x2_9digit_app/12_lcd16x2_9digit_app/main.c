/*
 * 12_lcd16x2_9digit_app.hex
 *
 * Created: 18-05-2026 23:19:26
 * Author : sreer
 * main.c
 * 9?digit counter with short/long press
 */
#define F_CPU 16000000UL
#include "lcd.h"
#include "timer.h"
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>
#define BUTTON PD2
int main(void)
{
    lcd_init();
    timer1_init();

    DDRD &= ~(1 << BUTTON);
    PORTD |= (1 << BUTTON);

    uint32_t counter = 0;
    lcd_set_cursor(0, 0);
    lcd_print_number(counter, 9);

    bool button_was_pressed = false;
    uint16_t press_start = 0;

    while (1)
    {
		    /* Replace with your application code */

        if (!(PIND & (1 << BUTTON)))
        {
            if (!button_was_pressed)
            {
                _delay_ms(30);                    // debounce
                if (!(PIND & (1 << BUTTON)))
                {
                    button_was_pressed = true;
                    press_start = timer1_ticks;   // record time
                }
            }
        }
        else
        {
            if (button_was_pressed)
            {
                _delay_ms(30);                    // debounce release
                if (PIND & (1 << BUTTON))
                {
                    uint16_t duration = timer1_ticks - press_start;
                    if (duration >= 2000)         // long press
                    {
                        counter = 0;
                    }
                    else                          // short press
                    {
                        counter++;
                        if (counter > 999999999) counter = 0;
                    }
                    lcd_set_cursor(0, 0);
                    lcd_print_number(counter, 9);
                    button_was_pressed = false;
                }
            }
        }
    }
}