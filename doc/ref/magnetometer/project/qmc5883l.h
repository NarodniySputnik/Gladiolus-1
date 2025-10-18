// qmc5883l.h
#ifndef QMC5883L_H_
#define QMC5883L_H_

#include <stdint.h>

// Основные режимы и настройки
#define QMC5883L_ADDRESS 0x0D

// Регистры QMC5883L
#define QMC5883L_REGISTER_X_LSB     0x00
#define QMC5883L_REGISTER_Y_LSB     0x02
#define QMC5883L_REGISTER_Z_LSB     0x04
#define QMC5883L_REGISTER_STATUS    0x06
#define QMC5883L_REGISTER_CONTROL1  0x09
#define QMC5883L_REGISTER_CONTROL2  0x0A

// Настройки режима работы
#define QMC5883L_MODE_STANDBY       0x00
#define QMC5883L_MODE_CONTINUOUS    0x01

// Частота выдачи данных (ODR)
#define QMC5883L_ODR_10HZ           0x00
#define QMC5883L_ODR_50HZ           0x01
#define QMC5883L_ODR_100HZ          0x02
#define QMC5883L_ODR_200HZ          0x03

// Диапазон измерений
#define QMC5883L_RANGE_2GAUSS       0x00
#define QMC5883L_RANGE_8GAUSS       0x01

// Внешние функции
void qmc5883l_init(void);
bool qmc5883l_get_data(int16_t *x, int16_t *y, int16_t *z);
float qmc5883l_get_temperature(void);
void qmc5883l_set_odr(uint8_t odr);
void qmc5883l_set_range(uint8_t range);
void qmc5883l_set_mode(uint8_t mode);

#endif /* QMC5883L_H_ */