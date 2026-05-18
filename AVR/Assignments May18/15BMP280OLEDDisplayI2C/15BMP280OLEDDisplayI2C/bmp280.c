/*
 * bmp280.c
 *
 * Created: 18-05-2026 23:48:31
 *  Author: sreer

 */

#include "bmp280.h"
#include "i2c.h"

#define BMP280_ADDR 0x76
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_TEMP_MSB 0xFA

uint8_t bmp280_init(void)
{
    i2c_start();
    i2c_write(BMP280_ADDR << 1);
    i2c_write(BMP280_REG_ID);
    i2c_start();
    i2c_write((BMP280_ADDR << 1) | 1);
    uint8_t id = i2c_read(0);
    i2c_stop();
    if (id != 0x58) return 0;

    // set temperature oversampling x1, normal mode
    i2c_start();
    i2c_write(BMP280_ADDR << 1);
    i2c_write(BMP280_REG_CTRL_MEAS);
    i2c_write(0x27);  // temp osrs=1, mode normal
    i2c_stop();

    // set standby time 0.5ms, filter off
    i2c_start();
    i2c_write(BMP280_ADDR << 1);
    i2c_write(BMP280_REG_CONFIG);
    i2c_write(0x00);
    i2c_stop();
    return 1;
}

float bmp280_read_temperature(void)
{
    uint8_t data[3];
    i2c_start();
    i2c_write(BMP280_ADDR << 1);
    i2c_write(BMP280_REG_TEMP_MSB);
    i2c_start();
    i2c_write((BMP280_ADDR << 1) | 1);
    data[0] = i2c_read(1);
    data[1] = i2c_read(1);
    data[2] = i2c_read(0);
    i2c_stop();
    int32_t adc_T = ((uint32_t)data[0] << 12) | ((uint16_t)data[1] << 4) | (data[2] >> 4);
    // Compensation formula (simplified with fixed calibration for demo)
    // Real code would read calibration registers. For this demonstration,
    // we assume typical values: dig_T1=27504, dig_T2=26435, dig_T3=-1000
    int32_t var1, var2, t_fine;
    var1 = ((((adc_T>>3) - ((int32_t)27504<<1))) * ((int32_t)26435)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)27504)) * ((adc_T>>4) - ((int32_t)27504))) >> 12) *
            ((int32_t)-1000)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) / 256 / 100.0;
}