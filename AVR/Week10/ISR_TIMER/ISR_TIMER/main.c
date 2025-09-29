/*
 * ISR_TIMER.c
 *
 * Created: 29-09-2025 20:56:27
 * Author : sreer
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
volatile uint16_t count=0;
ISR(TIMER0_OVF_vect)//Interrupt Service Routine (Address) Timer over flow.
{
		count++;

	if(count>977)
	{
		PORTB^=(1<<PORTB5);
		count=0;
	}

}

int main(void)
{
		sei();

    DDRB|=(1<<DDB5);
	//TCNT0=0;
	TCCR0A=0;//Normal Mode
	TCCR0B|=(1<<CS00)|(1<<CS01);//64 pre-scalar
	TIMSK0|=(1<<TOIE0);//enabling Timer Overflow Interupt timer 0 normal mode
	
	
	/* Replace with your application code */
    while (1) 
    {
    }
}

//HW hw: timer 0 overflow interrupt in ctc mode with OCR0A?

