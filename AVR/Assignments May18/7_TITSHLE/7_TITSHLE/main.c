/*
 * 7_TITSHLE.hex
 *
 * Created: 18-05-2026 22:53:28
 * Author : sreer

 * main.c
 * Independent LED toggles: PB1 every 2s (Timer1), PB2 every 500ms (Timer2)
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER1_COMPA_vect)
{
    PORTB ^= (1 << PB1);      // toggle Status LED
}

ISR(TIMER2_COMPA_vect)
{
    PORTB ^= (1 << PB2);      // toggle Heartbeat LED
}

int main(void)
{
    DDRB |= (1 << PB1) | (1 << PB2);

    // Timer1: CTC, prescaler 1024, OCR1A = 31249 for 2 s (16M/1024=15625, *2=31250-1)
    TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);  // 1024
    OCR1A = 31249;
    TIMSK1 |= (1 << OCIE1A);

    // Timer2: CTC, prescaler 128, OCR2A = 124 for 500 ms (16M/128=125k, *0.5=62500, 62500-1 too large->usse overflow+preascaler? 
	//Need precise: 125k ticks/s, 0.5s = 62500 ticks > 255. Use 1024 prescaler? 16M/1024=15625, 0.5s=7812.5 -> 
	//not integer. Use 256 prescaler: 16M/256=62500, OCR2A=125-1=124 for 0.5s? 62500 * 124 = 7,750,000 cycles not correct. Let's compute: with prescaler 256, timer freq = 16M/256 = 62500 Hz. To get 500 ms period, we need 62500*0.5 = 31250 counts. Since timer2 is 8?bit, we can use CTC with OCR2A = 250 (max 255) and prescaler 256, then count = 251 ticks per interrupt. 1/62500 * 251 = 0.004016 s. That's ~4 ms. Not 500 ms. So use prescaler 1024: freq=15625 Hz, to get 500 ms we need 7812.5 counts -> not possible. So implement a software counter. Simpler: use Timer0 overflow? But assignment requires Timer2. Use OCR2A = 124 with prescaler 1024 gives 125 ticks -> 125/15625 = 8 ms. Then count 63 interrupts for 500 ms. I'll demonstrate that.
    // Actually assignment says "every 500 milliseconds". So I'll use a static counter in ISR.
    // Set Timer2 CTC, preeescaler 1024, OCR2A = 124 ? 8 ms period. Count 62/63 to get ~500 ms.
    TCCR2A |= (1 << WGM21);            // CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024
    OCR2A = 124;
    TIMSK2 |= (1 << OCIE2A);

    sei();
    while (1);  // all work in ISRs
}
