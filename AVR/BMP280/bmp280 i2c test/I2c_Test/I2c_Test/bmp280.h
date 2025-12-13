#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include <stdbool.h>

bool bmp280_begin(uint8_t addr); // tries addr, 0x76, 0x77
bool bmp280_read(float *temperature_c, float *pressure_pa);
void bmp280_set_debug(bool en);

#endif // BMP280_H_
