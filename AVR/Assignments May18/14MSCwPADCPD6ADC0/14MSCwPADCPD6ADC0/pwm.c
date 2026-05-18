/*
 * pwm.c
 *
 * Created: 18-05-2026 23:40:16
 *  Author: sreer
 */ 
#include "pwm.h"
void pwm_init(void)
{
	DDRD |= (1 << DDD6);                // OC0A output
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);               // prescaler 8
	OCR0A = 0;                          // start off
}
void pwm_set_duty(uint8_t duty)
{
	OCR0A = duty;
}