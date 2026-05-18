/*
 * timer.c
 *
 * Created: 18-05-2026 22:45:44
 *  Author: sreer
*/

#include "timer.h"
#include <avr/interrupt.h>   

volatile static uint16_t timer1_ticks;

void timer1_init(void)
{
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
    OCR1A = 249;                       // 1 ms at 16 MHz
    TIMSK1 |= (1 << OCIE1A);
    timer1_ticks = 0;
    sei();                              // now the compiler knows sei()
}

void delay_ms_timer(uint16_t ms)
{
    timer1_ticks = 0;
    while (timer1_ticks < ms);
}

ISR(TIMER1_COMPA_vect)                 // now the ISR macro works correctly ID retturn Error vecrot _11
{
    timer1_ticks++;
}