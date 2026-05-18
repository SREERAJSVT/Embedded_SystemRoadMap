/*
 * 5TrafficLightControllerN0delay.c
 *
 * Created: 18-05-2026 22:25:28
 * Author : sreer

 * main.c
 * Traffic light: Green 10s, Orange 3s, Red 10s (timer?based)
 */
#include "timer.h"
#include <avr/io.h>

#define GREEN  PB0
#define ORANGE PB1
#define RED    PB2

int main(void)
{
    // Set LED pins as outputs
    DDRB |= (1 << GREEN) | (1 << ORANGE) | (1 << RED);

    // Initialize the timer module (starts 1 ms interrupts)
    timer1_init();

    while (1)
    {
        // Green ON for 10 seconds
        PORTB = (1 << GREEN);
        delay_ms_timer(10000);

        // Green OFF, Orange ON for 3 seconds
        PORTB = (1 << ORANGE);
        delay_ms_timer(3000);

        // Orange OFF, Red ON for 10 seconds
        PORTB = (1 << RED);
        delay_ms_timer(10000);
    }
}