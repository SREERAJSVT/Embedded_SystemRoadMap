/*
 * 4_BitBinaryCounterwithButton.hex
 *
 * Created: 18-05-2026 22:22:54
 * Author : sreer
 * main.c
 * 4?bit binary counter on PB0-PB3, button on PD2
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define BUTTON PD2

void updateLEDs(uint8_t count)
{
    PORTB = (PORTB & 0xF0) | (count & 0x0F);  // preserve upper nibble
}

int main(void)
{
    DDRB |= 0x0F;                    // PB0-3 outputs
    DDRD &= ~(1 << BUTTON);
    PORTD |= (1 << BUTTON);          // pull-up

    uint8_t counter = 0;
    updateLEDs(counter);

    while (1)
    {
        if (!(PIND & (1 << BUTTON)))
        {
            _delay_ms(30);
            if (!(PIND & (1 << BUTTON)))
            {
                if (counter < 15)
                    counter++;
                else
                    counter = 0;
                updateLEDs(counter);
                while (!(PIND & (1 << BUTTON)));
                _delay_ms(30);
            }
        }
    }
}

