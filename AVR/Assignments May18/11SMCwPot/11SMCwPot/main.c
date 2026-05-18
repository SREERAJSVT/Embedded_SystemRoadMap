/*
 * 11SMCwPot.c
 *
 * Created: 18-05-2026 23:10:41
 * Author : sreer
 */ 

#include <avr/io.h>
/*
 * main.c
 */

#include "adc.h"
#include "servo.h"
#include <util/delay.h>

int main(void)
{
    adc_init();
    servo_init();

    while (1)
    {
		    /* Replace with your application code */

        uint16_t pot = adc_read(0);
        servo_set_angle(pot);
        _delay_ms(20);   // small update period
    }
}
    