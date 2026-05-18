/*
 * timer.c
 *
 * Created: 18-05-2026 22:36:33
 *  Author: sreer
 * main.c
 * Traffic light: Green 10s, Orange 3s, Red 10s (timer?based)
 *V0.1 Updated  Timer1 driver – NO main() function here!
 */

#include "timer.h"
#include <avr/interrupt.h>

// Global tick counter – MUST be volatile
volatile static uint16_t timer1_ticks;

void timer1_init(void)
{
    // CTC mode, prescaler 64 ? 250 kHz, OCR1A = 249 ? 1 ms interrupt
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 249;                     // (16M/64)/1000 - 1 = 249
    TIMSK1 |= (1 << OCIE1A);
    timer1_ticks = 0;
    sei();                           // enable global interrupts
}

void delay_ms_timer(uint16_t ms)
{
    timer1_ticks = 0;
    while (timer1_ticks < ms);       // wait until enough ticks passed
}

ISR(TIMER1_COMPA_vect)
{
    timer1_ticks++;                  // increment every 1 ms
}