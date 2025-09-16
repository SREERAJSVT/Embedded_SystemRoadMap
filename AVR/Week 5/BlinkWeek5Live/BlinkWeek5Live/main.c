/*
 * BlinkWeek5Live.c
 *
 * Created: 16-09-2025 23:11:06
 * Author : Sreeraj Krishna K
 */ 
#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>


int main(void)
{
	    DDRB = 0b00100000; // 0 means input , 1 means output.
    /* Replace with your application code */
    while (1) 
    {
        PORTB = 0b00100000;
				_delay_ms(1000);
        PORTB = 0b00000000;
				_delay_ms(1000);

		
    }
}

