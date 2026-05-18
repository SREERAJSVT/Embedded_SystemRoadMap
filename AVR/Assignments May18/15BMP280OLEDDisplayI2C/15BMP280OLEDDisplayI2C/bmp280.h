/*
 * bmp280.h
 *
 * Created: 18-05-2026 23:47:49
 *  Author: sreer 
 * bmp280.h / bmp280.c  (simplified for temperature only)
 */ 

#ifndef BMP280_H_
#define BMP280_H_

#include <avr/io.h>

uint8_t bmp280_init(void);
float bmp280_read_temperature(void);

#endif