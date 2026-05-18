/*
 * PWMBCB.c
 *
 * Created: 18-05-2026 22:57:13
 * Author : sreer
  * Button cycles duty: 25% ? 50% ? 75% ? OFF ? 25%...

 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include "pwm.h"
#include <util/delay.h>
#define BUTTON PD2
int main(void)
{
    pwm_init();
    DDRD &= ~(1 << BUTTON);
    PORTD |= (1 << BUTTON);    // pull?up
    const uint8_t duties[] = {64, 128, 192, 0};  // 25%, 50%, 75%, OFF
    uint8_t idx = 0;
    pwm_set_duty(duties[idx]);
    while (1)
    {     /* Replace with your application code */

        if (!(PIND & (1 << BUTTON)))
        {
            _delay_ms(30);                     // debouncea
            if (!(PIND & (1 << BUTTON)))
            {
                idx++;
                if (idx > 3) idx = 0;
                pwm_set_duty(duties[idx]);

                while (!(PIND & (1 << BUTTON))); // wait release.
                _delay_ms(30);
            }
        }
    }
}
