/*
 * Week6.c
 *
 * Created: 17-09-2025 00:18:46
 * Author : sreeraj
 */ 
#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRB=0b00100000;
	DDRC=0b00000000;
    /* Replace with your application code */
    while (1) 
    {
		if (PIND==0b0000001)
		 {
			 
		 PORTB=0b00100000;
		 
		 }
		 else
		 {
		PORTB=0b00000000;

		 }

		 
		
    }
}

