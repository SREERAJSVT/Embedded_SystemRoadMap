/*
 * adc.h
 *
 * Created: 18-05-2026 23:04:17
 *  Author: sreer
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void adc_init(void);
uint16_t adc_read(uint8_t channel);



#endif /* ADC_H_ */