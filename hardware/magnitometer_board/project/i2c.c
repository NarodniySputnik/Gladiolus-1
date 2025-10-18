// i2c.c
#include <avr/io.h>
#include <util/twi.h>
#include "i2c.h"

// Функция инициализации I²C-шины
void i2c_init(void) {
    // Настройка тактовой частоты I²C
    TWBR = ((F_CPU / I2C_CLOCK_SPEED) - 16) / 2;
    TWSR = 0x00; // Pre-scaler = 1
}

// Генерирует условие START на шине I²C
void i2c_start(uint8_t addr) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT); // Генерация условия START
    while (!(TWCR & (1<<TWINT))) ; // Ждём завершения условия START
    TWDR = addr; // Устанавливаем адрес устройства
    TWCR = (1<<TWEN)|(1<<TWINT); // Готовим передачу адреса
    while (!(TWCR & (1<<TWINT))) ; // Ждём подтверждения передачи адреса
}

// Генерирует условие STOP на шине I²C
void i2c_stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN); // Генерация условия STOP
    while (TWCR & (1<<TWSTO)) ; // Ждём завершения условия STOP
}

// Отправляет байт по I²C
void i2c_write(uint8_t byte) {
    TWDR = byte; // Загружаем байт в регистр данных
    TWCR = (1<<TWEN)|(1<<TWINT); // Запускаем передачу
    while (!(TWCR & (1<<TWINT))) ; // Ждём завершения передачи
}

// Читает байт с I²C
uint8_t i2c_read(bool ack) {
    TWCR = (1<<TWEN)|(1<<TWINT)|(ack<<TWEA); // ACK = 1 для продолжения, NACK = 0 для последнего байта
    while (!(TWCR & (1<<TWINT))) ; // Ждём завершения чтения
    return TWDR; // Возвращаем прочитанное значение
}