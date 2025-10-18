// pcf8563.h
#ifndef PCF8563_H_
#define PCF8563_H_

#include <stdint.h>
#include <avr/io.h>

// Адрес PCF8563 на шине I²C
#define PCF8563_ADDRESS 0x51

// Функции для работы с PCF8563
void pcf8563_init(void);
void pcf8563_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void pcf8563_set_date(uint8_t day, uint8_t month, uint8_t year);
void pcf8563_set_alarm_interval(uint32_t seconds);
uint32_t pcf8563_get_unix_time(void);

#endif /* PCF8563_H_ */