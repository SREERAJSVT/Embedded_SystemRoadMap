/*
 * rtc.h
 *
 *  Created on: 27-Dec-2025
 *      Author: sreer
 */

#ifndef RTC_H_
#define RTC_H_

#include <stdint.h>

typedef struct {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
  uint8_t day;
  uint8_t date;
  uint8_t month;
  uint8_t year;
} RTC_Time;

void RTC_Init(void);
void RTC_GetTime(RTC_Time *time);
void RTC_SetTime(RTC_Time *time);

#endif /* RTC_H_ */
