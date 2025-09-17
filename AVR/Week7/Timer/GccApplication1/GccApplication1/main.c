/*
 * GccApplication1.c
 *
 * Created: 17-09-2025 20:48:45
 * Author : sreer
 */ 
#define F_CPU 16000000L
#include <avr/io.h>
void delay_1s(void)
{
	unsigned int i;
	unsigned int count=62500;
	for (i=0;i<count;i++)
	{
	TCNT0=0;
	TCCR0A=0;
	TCCR0A|=(1<<CS00);
	while (!(TIFR0 &(1<<TOV0)))
	{
		
	}

}
}

int main(void)
{
    /* Replace with your application code */
    while (1)
	{ 
		PORTB |=(1<<PORTB5);
		delay_1s();
		PORTB &=~(1<<PORTB5);
		delay_1s();
		//portb^=(1<<PORTB5); xor
	}
}

