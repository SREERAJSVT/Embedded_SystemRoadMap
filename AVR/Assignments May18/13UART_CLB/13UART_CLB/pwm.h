/*
 * pwm.h
 *
 * Created: 18-05-2026 22:57:35
 *  Author: sreer
 */
#ifndef PWM_H_
#define PWM_H_
#include <avr/io.h>
void pwm_init(void);
void pwm_set_duty(uint8_t duty);  // 0-255
#endif /* PWM_H_ */