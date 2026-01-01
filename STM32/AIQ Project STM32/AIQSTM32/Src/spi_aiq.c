#include "spi_ll.h"
#include "stm32f4xx_ll_spi.h"
#include "board_pins.h"
#include "stm32f4xx_ll_gpio.h"

void spi1_cs_low(void)  { LL_GPIO_ResetOutputPin(SD_CS_PORT, SD_CS_PIN); }
void spi1_cs_high(void) { LL_GPIO_SetOutputPin(SD_CS_PORT, SD_CS_PIN); }

uint8_t spi1_txrx(uint8_t v) {
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
    LL_SPI_TransmitData8(SPI1, v);
    while(!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
    return LL_SPI_ReceiveData8(SPI1);
}
