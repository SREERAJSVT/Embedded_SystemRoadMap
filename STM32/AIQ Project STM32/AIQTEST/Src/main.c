#include "oled_ssd1306.h"
#include "rtc.h"
#include "sd_card.h"
#include "sensirion_voc_algorithm.h" // Needed for VocAlgorithmParams
#include "sgp40.h"
#include "stm32f446xx.h"
#include <stdio.h>

/* Function Prototypes - Resolves "implicit declaration" warnings */
void System_Init(void);
void SHT40_Read(float *t, float *rh);
// SGP40_GetRaw is in sgp40.h
float DewPoint_Compute(float t, float rh);
void SD_Log_Data(float t, float rh, int32_t voc, float dp);
void UART2_SendString(char *str);
void UART2_SendChar(char c);

/* Global Variables - Resolves 'voc_params' undeclared */
VocAlgorithmParams voc_params;

int main(void) {
  System_Init();
  VocAlgorithm_init(&voc_params);
  OLED_Init();
  RTC_Init();
  SGP40_Init();

  UART2_SendString("--- AIQ System Booting ---\r\n");

  if (SD_Init() == 0) {
    UART2_SendString("SD Card Initialized\r\n");
  } else {
    UART2_SendString("SD Card Failed\r\n");
  }

  char debug_buf[100];
  float t, rh;
  RTC_Time currentTime;
  int32_t voc_index = 0;

  while (1) {
    SHT40_Read(&t, &rh);
    RTC_GetTime(&currentTime);

    // Calculate VOC Index
    uint16_t sraw = SGP40_GetRaw(t, rh);
    voc_index = VocAlgorithm_process(&voc_params, sraw);

    OLED_Display_Metrics(&currentTime, t, rh, voc_index);

    // Print values to Serial Monitor
    printf(debug_buf, "[%02d:%02d:%02d] T: %.1f C, RH: %.1f %%, VOC: %ld\r\n",
            currentTime.hours, currentTime.minutes, currentTime.seconds, t, rh,
            voc_index);
    UART2_SendString(debug_buf);

    // Log to SD Card
    SD_Log_Data(t, rh, voc_index, 0.0f);

    for (volatile int i = 0; i < 1000000; i++)
      ; // Delay
  }
}

void UART2_SendChar(char c) {
  while (!(USART2->SR & (1 << 7)))
    ; // Wait for TXE (Transmit buffer empty)
  USART2->DR = c;
}

void UART2_SendString(char *str) {
  while (*str)
    UART2_SendChar(*str++);
}

void SD_Log_Data(float t, float rh, int32_t voc, float dp) {
  char buf[64];
  // Format: T,RH,VOC
  sprintf(buf, "%.2f,%.2f,%ld", t, rh, voc);
  SD_Log(buf);
}
