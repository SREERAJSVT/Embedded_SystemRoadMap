/*
 * 14MSCwPADCPD6ADC0.HEX
 *Potentiometer ADC0
 Motor driver input PD6 (OC0A)
 * Created: 18-05-2026 23:38:22
 * Author : sreer
 */ 
//#define F_CPU=16000000UL
#include <avr/io.h>
#include "adc.h"
#include "pwm.h"
#include <util/delay.h>
int main(void)
{    /* Replace with your application code */

    adc_init();
    pwm_init();

    while (1)
    {
        uint16_t raw = adc_read(0);
        uint8_t speed = (uint8_t)((uint32_t)raw * 255 / 1023);
        pwm_set_duty(speed);
        _delay_ms(20);
    }
}