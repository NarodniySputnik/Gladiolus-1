// i2c.h
#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

// Базовая скорость шины I²C
#define I2C_CLOCK_SPEED 400000UL // 400 kHz

// Основные функции I²C
void i2c_init(void);
void i2c_start(uint8_t addr);
void i2c_stop(void);
void i2c_write(uint8_t byte);
uint8_t i2c_read(bool ack);

#endif /* I2C_H_ */