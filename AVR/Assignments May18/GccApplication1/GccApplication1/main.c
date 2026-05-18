/*
 * GccApplication1.c
 *
 * Created: 18-05-2026 22:03:46
 * Author : sreer
 */ 

/*
 * main.c
 * Toggle PD5: 3 s ON, 5 s OFF
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRD |= (1 << DDD5);          // PD5 output

    while (1)
    {
        PORTD |= (1 << PORTD5);   // LED ON
        _delay_ms(3000);
        PORTD &= ~(1 << PORTD5);  // LED OFF
        _delay_ms(5000);
    }
}

