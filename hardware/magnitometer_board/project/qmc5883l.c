// qmc5883l.c
#include <avr/io.h>
#include <util/i2c.h>
#include "i2c.h"
#include "qmc5883l.h"

// Функция инициализации QMC5883L
void qmc5883l_init(void) {
    // Настройка контроллеров магнитометра
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL1); // Регистр CONTROL1
    i2c_write(QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_50HZ); // Непрерывный режим, частота 50 Гц
    i2c_stop();

    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL2); // Регистр CONTROL2
    i2c_write(QMC5883L_RANGE_2GAUSS); // Диапазон ±2 Гаусса
    i2c_stop();
}

// Функция чтения данных X,Y,Z оси
bool qmc5883l_get_data(int16_t *x, int16_t *y, int16_t *z) {
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_X_LSB); // Начинать читать с младшего бита X
    i2c_start(QMC5883L_ADDRESS << 1 | 1); // Чтение
    *x = (i2c_read(true) << 8) | i2c_read(true); // X
    *y = (i2c_read(true) << 8) | i2c_read(true); // Y
    *z = (i2c_read(true) << 8) | i2c_read(false); // Z
    i2c_stop();
    return true;
}

// Функция получения температуры
float qmc5883l_get_temperature(void) {
    uint8_t temperature_raw;
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_STATUS); // Регистр STATUS
    i2c_start(QMC5883L_ADDRESS << 1 | 1); // Чтение
    temperature_raw = i2c_read(false); // Читаем статус
    i2c_stop();

    // Предполагается, что статус содержит индикатор перегрева, реальная температура тут не читается
    return 25.0; // Используем заглушку
}

// Функция установки выходной частоты (Output Data Rate)
void qmc5883l_set_odr(uint8_t odr) {
    uint8_t reg_value;
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL1); // Читаем текущий регистр
    i2c_start(QMC5883L_ADDRESS << 1 | 1);
    reg_value = i2c_read(false);
    i2c_stop();

    // Маска для удаления старых битов частоты
    reg_value &= ~0x03; // Удаляем старые биты частоты
    reg_value |= odr & 0x03; // Присваиваем новые биты частоты

    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL1); // Записываем обратно
    i2c_write(reg_value);
    i2c_stop();
}

// Функция установки диапазона измерений
void qmc5883l_set_range(uint8_t range) {
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL2); // Регистр диапазонов
    i2c_write(range); // Устанавливаем нужный диапазон
    i2c_stop();
}

// Функция переключения режима работы
void qmc5883l_set_mode(uint8_t mode) {
    uint8_t reg_value;
    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL1); // Читаем текущий регистр
    i2c_start(QMC5883L_ADDRESS << 1 | 1);
    reg_value = i2c_read(false);
    i2c_stop();

    // Маска для удаления старых битов режима
    reg_value &= ~0x01; // Удаляем старый бит режима
    reg_value |= mode & 0x01; // Применяем новый режим

    i2c_start(QMC5883L_ADDRESS << 1);
    i2c_write(QMC5883L_REGISTER_CONTROL1); // Записываем обратно
    i2c_write(reg_value);
    i2c_stop();
}