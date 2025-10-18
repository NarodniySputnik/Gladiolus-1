// pcf8563.c
#include <avr/io.h>
#include <util/i2c.h>
#include "i2c.h"
#include "pcf8563.h"

// Вспомогательная функция для декодирования BCD (Binary pcf8563.c
#include Coded Decimal)
static inline uint8_t bcd_decode(uint8_t value) {
    return ((value >> 4) * 10) + (value & 0x0F);
}

// Вспомогательная функция для кодирования BCD
static inline uint8_t bcd_encode(uint8_t value) {
    return ((value 4) | (value % 10);
}

// Инициализация PCF8563
void pcf8563_init(void) {
    i2c_start(PCF8563_ADDRESS << 1); // Адрес устройства
    i2c_write(0x00); // Первый регистр (контрольный)
    i2c_write(0x00); // Сброс регистра
    i2c_stop();
}

// Установка текущего времени
void pcf8563_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    i2c_start(PCF8563_ADDRESS << 1);
    i2c_write(0x03); // Переходим к регистру времени
    i2c_write(bcd_encode(seconds)); // Устанавливаем секунды
    i2c_write(bcd_encode(minutes)); // Устанавливаем минуты
    i2c_write(bcd_encode(hours)); // Устанавливаем часы
    i2c_stop();
}

// Установка текущей даты
void pcf8563_set_date(uint8_t day, uint8_t month, uint8_t year) {
    i2c_start(PCF8563_ADDRESS << 1);
    i2c_write(0x05); // Переходим к регистру даты
    i2c_write(bcd_encode(day)); // Устанавливаем день
    i2c_write(bcd_encode(month)); // Устанавливаем месяц
    i2c_write(bcd_encode(year)); // Устанавливаем год
    i2c_stop();
}

// Установка интервала будильника (упрощённая версия)
void pcf8563_set_alarm_interval(uint32_t seconds) {
    // Данная реализация ограничивает интервал до максимального периода будильника
    // Будильник PCF8563 ограничен периодом 24 часа (не больше суток)
    uint8_t secs_bcd = bcd_encode(seconds % 60);
    uint8_t mins_bcd = bcd_encode((seconds / 60) % 60);
    uint8_t hrs_bcd = bcd_encode((seconds / 3600) % 24);

    i2c_start(PCF8563_ADDRESS << 1);
    i2c_write(0x0E); // Начало области будильников
    i2c_write(secs_bcd); // Вторую секунду будильника игнорируем
    i2c_write(mins_bcd); // Минуты будильника
    i2c_write(hrs_bcd); // Часы будильника
    i2c_stop();
}

// Получение текущего времени в виде UNIX timestamp
uint32_t pcf8563_get_unix_time(void) {
    uint8_t second, minute, hour, day, month, year;

    i2c_start(PCF8563_ADDRESS << 1);
    i2c_write(0x03); // Регистр времени
    i2c_start(PCF8563_ADDRESS << 1 | 1); // Чтение
    second = i2c_read(true); // Читаем секунды
    minute = i2c_read(true); // Читаем минуты
    hour = i2c_read(true); // Читаем часы
    day = i2c_read(true); // Читаем день
    month = i2c_read(true); // Читаем месяц
    year = i2c_read(false); // Читаем год

    i2c_stop();

    // Декодируем значения из BCD
    second = bcd_decode(second);
    minute = bcd_decode(minute);
    hour = bcd_decode(hour);
    day = bcd_decode(day);
    month = bcd_decode(month);
    year += 2000; // Предположительно год указан от 2000

    // Вычисляем UNIX timestamp вручную
    struct tm timeinfo = {
        .tm_year = year - 1900,
        .tm_mon = month - 1,
        .tm_mday = day,
        .tm_hour = hour,
        .tm_min = minute,
        .tm_sec = second
    };
    return mktime(&timeinfo);
}