/*
 * AC_LM35_DCM.c
 *
 * Created: 18-05-2026 23:03:38
 * Author : sreer
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"
#include "pwm.h"
#define BUTTON PD2

float read_temperature(void)
{
	uint16_t raw = adc_read(0);
	// LM35: 10 mV/°C, Vref=5V, ADC 10-bit ? 4.88 mV/LSB
	// temp = raw * 0.488;
	return raw * 0.488;
}

void set_motor_speed(float temp)
{
	uint8_t duty;
	if (temp < 25.0)
	duty = 76;      // ~30% of 255
	else if (temp < 35.0)
	duty = 178;     // ~70%
	else
	duty = 255;     // 100%
	pwm_set_duty(duty);
}

int main(void)
{
	    /* Replace with your application code */

	adc_init();
	pwm_init();
	DDRD &= ~(1 << BUTTON);
	PORTD |= (1 << BUTTON);

	uint8_t system_on = 1;  // start ON
	while (1)
	{
		if (!(PIND & (1 << BUTTON)))
		{
			_delay_ms(30);
			if (!(PIND & (1 << BUTTON)))
			{
				system_on = !system_on;
				if (!system_on)
				pwm_set_duty(0);    // motor off
				while (!(PIND & (1 << BUTTON)));
				_delay_ms(30);
			}
		}

		if (system_on)
		{
			float temp = read_temperature();
			set_motor_speed(temp);
		}
		_delay_ms(100);
	}
}
