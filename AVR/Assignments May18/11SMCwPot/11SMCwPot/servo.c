/*
 * servo.c
 *
 * Created: 18-05-2026 23:11:22
 *  Author: sreer
 */ 
#include "servo.h"
void servo_init(void)
{
    DDRB |= (1 << PB1);               // OC1A output
    // Phase Correct PWM, Prescale 8, ICR1 = 39999 ? 50 Hz
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << CS11);
    ICR1 = 39999;
    OCR1A = 1500;                     // centre
}

void servo_set_angle(uint16_t adc_value)
{
    // Map 0-1023 to 1000-2000 µs pulse
    // With ICR1=39999, 1 µs = 2 counts
    uint16_t pulse = 1000 + (uint32_t)adc_value * 1000 / 1023;  // 1000-2000 µs
    OCR1A = pulse * 2;
}