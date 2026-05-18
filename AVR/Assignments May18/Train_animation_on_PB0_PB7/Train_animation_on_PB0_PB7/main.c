/*
 * Train_animation_on_PB0_PB7.hex
 *
 * Created: 18-05-2026 22:20:23
 * Author : sreer
 * main.c
 * Train animation on PB0 - PB7
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRB = 0xFF;               // all PORTB outputs
    uint8_t pattern = 0x01;

    while (1)
    {
        PORTB = pattern;
        _delay_ms(200);
        pattern <<= 1;
        if (pattern == 0)
            pattern = 0x01;    // wrap around
    }
}

