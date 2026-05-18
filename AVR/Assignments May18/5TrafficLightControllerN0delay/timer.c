/*
 * timer.c
 */

#include "timer.h"

volatile static uint16_t timer1_ticks;

void timer1_init(void)
{
    // CTC mode, prescaler 64 → 250 kHz, OCR1A = 250 → 1 ms interrupt
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 249;                 // (16M/64)/1000 - 1 = 249
    TIMSK1 |= (1 << OCIE1A);
    timer1_ticks = 0;
    sei();
}

void delay_ms_timer(uint16_t ms)
{
    timer1_ticks = 0;
    while (timer1_ticks < ms);
}

ISR(TIMER1_COMPA_vect)
{
    timer1_ticks++;
}