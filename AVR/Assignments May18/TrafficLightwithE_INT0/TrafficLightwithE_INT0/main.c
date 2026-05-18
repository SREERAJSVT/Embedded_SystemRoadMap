/*
 * TrafficLightwithE_INT0.hex
 *
 * Created: 18-05-2026 22:45:28
 * Author : sreer
 Traffic light with INT0 emergency override

 */ 
#include "timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define GREEN  PB0
#define ORANGE PB1
#define RED    PB2

volatile uint8_t emergency_flag = 0;

ISR(INT0_vect)
{
    emergency_flag = 1;
    // No immediate state change
}

void set_lights(uint8_t green, uint8_t orange, uint8_t red)
{
    PORTB = (green  ? (1<<GREEN)  : 0) |
            (orange ? (1<<ORANGE) : 0) |
            (red    ? (1<<RED)    : 0);
}

int main(void)
{
    DDRB |= (1<<GREEN) | (1<<ORANGE) | (1<<RED);
    // INT0 setup: falling edge
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);
    // enable pull?up on PD2
    PORTD |= (1 << PD2);

    timer1_init();   // already enables global interrupts

    while (1)
    {
        // Normal Green
        set_lights(1,0,0);
        delay_ms_timer(10000);

        // Check emergency flag after Green finishes
        if (emergency_flag)
        {
            // Prioritise Green for 10 s
            set_lights(1,0,0);
            delay_ms_timer(10000);
            emergency_flag = 0;
        }
        else
        {
            // Orange
            set_lights(0,1,0);
            delay_ms_timer(3000);

            if (emergency_flag)
            {
                set_lights(1,0,0);
                delay_ms_timer(10000);
                emergency_flag = 0;
            }
            else
            {
                // Red
                set_lights(0,0,1);
                delay_ms_timer(10000);
            }
        }
    }
}
