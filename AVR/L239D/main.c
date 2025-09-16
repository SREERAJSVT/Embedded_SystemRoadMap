/*
 * GccApplication1.c
 *
 * Created: 15-09-2025 21:32:36
 * Author : sreer
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include<util/delay.h>
void cw()
{
	PORTB|=(1<<PORTB5);
	PORTB&=~(1<<PORTB4);
	_delay_ms(3000);
}
void stp()
{
	PORTB&=~(1<<PORTB5);
	PORTB&=~(1<<PORTB4);
	_delay_ms(2000);
}
void ccw()
{
	PORTB&=~(1<<PORTB5);
	PORTB|=(1<<PORTB4);
	_delay_ms(3000);
}
int main(void)
{
    DDRB|=(1<<DDB5);
	DDRB|=(1<<DDB4);
    while (1) 
    {
		cw();//CALL CLOCKWISE
		_delay_ms(30000);//Debug Purspose.
		stp();//call stop
		ccw();//call counter clockwise
		stp();//call stop
    }
}
