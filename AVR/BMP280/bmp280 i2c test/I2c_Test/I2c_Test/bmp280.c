#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include "bmp280.h"
#include "i2c.h"
#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BMP280_REG_DIG_T1    0x88
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

#define BMP280_CHIP_ID 0x58
#define BMP280_RESET_VALUE 0xB6

static uint8_t bmp_addr = 0x76;
static bool bmp_connected = false;
static bool bmp_debug = false;

static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2;
static int16_t  dig_P3;
static int16_t  dig_P4;
static int16_t  dig_P5;
static int16_t  dig_P6;
static int16_t  dig_P7;
static int16_t  dig_P8;
static int16_t  dig_P9;

static int32_t t_fine;

void bmp280_set_debug(bool en) { bmp_debug = en; }

static void dbg(const char *s)
{
	if (bmp_debug) uart_puts(s);
}

static bool read_coeffs(void)
{
	uint8_t buf[24];
	if (!i2c_read_registers(bmp_addr, BMP280_REG_DIG_T1, buf, 24)) return false;

	dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
	dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
	dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
	dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
	dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
	dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
	dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
	dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
	dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
	dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
	dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
	dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

	if (bmp_debug) {
		char t[128];
		snprintf(t, sizeof(t), "cal T1=%u T2=%d T3=%d P1=%u P2=%d P3=%d\r\n",
		dig_T1, dig_T2, dig_T3, dig_P1, dig_P2, dig_P3);
		uart_puts(t);
	}
	return true;
}

bool bmp280_begin(uint8_t addr)
{
	i2c_init();

	uint8_t addrs[3] = { addr, 0x76, 0x77 };
	bool found = false;
	for (uint8_t i = 0; i < 3; ++i) {
		uint8_t a = addrs[i];
		uint8_t id;
		if (!i2c_read_registers(a, BMP280_REG_ID, &id, 1)) continue;
		if (id == BMP280_CHIP_ID) { bmp_addr = a; found = true; break; }
	}
	if (!found) { bmp_connected = false; dbg("BMP280 not found\r\n"); return false; }

	if (!read_coeffs()) { bmp_connected = false; dbg("coeff read fail\r\n"); return false; }

	// config
	i2c_write_register(bmp_addr, BMP280_REG_CONFIG, (5 << 5)); // t_sb=1000ms
	i2c_write_register(bmp_addr, BMP280_REG_CTRL_MEAS, (1 << 5) | (1 << 2) | 3); // osrs_t=1 osrs_p=1 normal mode

	bmp_connected = true;
	if (bmp_debug) {
		char s[40]; snprintf(s, sizeof(s), "BMP280 @0x%02X\r\n", bmp_addr); uart_puts(s);
	}
	return true;
}

static float compensate_T(int32_t adc_T)
{
	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	float T = (t_fine * 5 + 128) >> 8;
	return T / 100.0f;
}

static float compensate_P(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) return 0;
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (float)(p >> 8);
}

bool bmp280_read(float *temperature_c, float *pressure_pa)
{
	if (!bmp_connected) return false;

	uint8_t data[6];
	if (!i2c_read_registers(bmp_addr, BMP280_REG_PRESS_MSB, data, 6)) return false;

	int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);
	int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((data[5] >> 4) & 0x0F);

	if (bmp_debug) {
		char tmp[64];
		snprintf(tmp, sizeof(tmp), "rawT=%ld rawP=%ld\r\n", (long)adc_T, (long)adc_P);
		uart_puts(tmp);
	}

	*temperature_c = compensate_T(adc_T);
	*pressure_pa = compensate_P(adc_P);

	if (bmp_debug) {
		char tmp[64];
		snprintf(tmp, sizeof(tmp), "T=%.2fC P=%.2fPa\r\n", *temperature_c, *pressure_pa);
		uart_puts(tmp);
	}
	return true;
}
