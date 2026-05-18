/*
 * adc.c
 *
 * Created: 18-05-2026 23:05:37
 *  Author: sreer
 */ 
#include "adc.h"

void adc_init(void)
{
	ADMUX = (1 << REFS0);           // AVcc reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // prescaler 64
}

uint16_t adc_read(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}