/*
 * main.c
 * ATmega328P Blink with 1s delay using Timer1
 * Author : Sreeraj Krishna K
 */

#define F_CPU 16000000UL     // 16 MHz clock
#include <avr/io.h>

void delay_1s(void)
{
    // Set prescaler = 1024
    TCCR1B |= (1 << CS12) | (1 << CS10);  

    // Reset counter
    TCNT1 = 0;                           

    // Wait until Timer1 counts 15625 ticks = 1 second
    while (TCNT1 < 15625) ;              

    // Stop Timer1
    TCCR1B = 0;                          
}

int main(void)
{
    // Set PB5 (Arduino digital pin 13) as output
    DDRB |= (1 << DDB5);

    while (1)
    {
        PORTB ^= (1 << PORTB5);  // Toggle LED
        delay_1s();              // Wait 1 second
    }
}
