/*
 * 2RGLEDTg.hex
 *
 * Created: 18-05-2026 22:17:57
 * Author : sreer
 * main.c
 * Toggle Red/Green LED on PB0/PB1 with button on PD2
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define RED_LED    PB0
#define GREEN_LED  PB1
#define BUTTON     PD2

int main(void)
{
    // LED outputs
    DDRB |= (1 << RED_LED) | (1 << GREEN_LED);
    // Button input with internal pull-up
    DDRD &= ~(1 << BUTTON);
    PORTD |= (1 << BUTTON);

    uint8_t state = 0;   // 0: Red ON, 1: Green ON
    PORTB |= (1 << RED_LED);
    PORTB &= ~(1 << GREEN_LED);

    while (1)
    {
        if (!(PIND & (1 << BUTTON)))          // pressed (active low)
        {
            _delay_ms(20);                   // debounce
            if (!(PIND & (1 << BUTTON)))
            {
                state = !state;              // toggle
                if (state)
                {
                    PORTB &= ~(1 << RED_LED);
                    PORTB |= (1 << GREEN_LED);
                }
                else
                {
                    PORTB |= (1 << RED_LED);
                    PORTB &= ~(1 << GREEN_LED);
                }
                // wait for release
                while (!(PIND & (1 << BUTTON)));
                _delay_ms(20);
            }
        }
    }
}
