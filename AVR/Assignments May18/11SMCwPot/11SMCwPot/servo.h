/*
 * servo.h
 *
 * Created: 18-05-2026 23:11:04
 *  Author: sreer
 */ 

#ifndef SERVO_H_
#define SERVO_H_

#include <avr/io.h>

void servo_init(void);
void servo_set_angle(uint16_t adc_value);

#endif