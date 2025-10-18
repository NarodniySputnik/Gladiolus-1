// main.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "uart.h"
#include "i2c.h"
#include "pcf8563.h"
#include "qmc5883l.h"
#include "ff.h"

// Макросы и дефолтные настройки
#define DEFAULT_ALARM_INTERVAL 120
#define BAUD_RATE 9600

// Буферы и переменные
#define UART_BUFFER_SIZE 128
volatile char uart_rx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_rx_head = 0;
volatile uint8_t uart_rx_tail = 0;

// Текущие настройки
static uint32_t alarm_interval_sec = DEFAULT_ALARM_INTERVAL;

// Переменные для работы с файлами
FIL log_file; // Журнал измерений
FIL config_file; // Файл конфигурации

FATFS fs;

volatile bool rtc_alarm_flag = false;
volatile bool uart_command_ready = false;

// Логирование в файл log.txt
static void log_measurement(void) {
    FRESULT res;
    int16_t x, y, z;
    float temp_mag, temp_mcu;

    // Получить UNIX время
    uint32_t unix_time = pcf8563_get_unix_time();

    // Считать магнитометр
    if (!qmc5883l_get_data(&x, &y, &z)) {
        return;
    }
    temp_mag = qmc5883l_get_temperature();

    // Измеряем внутреннюю температуру MCU
    temp_mcu = measure_internal_temp();

    char line[128];
    int len = snprintf(line, sizeof(line), "%lu,%d,%d,%d,%.2f,%.2f\r\n",
                       unix_time, x, y, z, temp_mag, temp_mcu);

    res = f_open(&log_file, "log.txt", FA_WRITE | FA_OPEN_APPEND);
    if (res == FR_OK) {
        f_lseek(&log_file, f_size(&log_file));
        UINT bw;
        f_write(&log_file, line, len, &bw);
        f_close(&log_file);
    }
}

// Функция измерения температуры MCU
static float measure_internal_temp(void) {
    ADMUX = INTERNAL_TEMP_SENSOR_ENABLE; // Включаем внутренний термодатчик
    _delay_us(100); // Пауза для стабилизации
    ADCSRA |= _BV(ADSC); // Начинаем преобразование
    while(!(ADCSRA & _BV(ADIF))); // Ждём завершения преобразования
    ADCSRA |= _BV(ADIF); // Сбрасываем флаг завершения преобразования
    ADCSRA &= ~(_BV(ADEN)); // Выключение АЦП после измерения
    return (ADCW - 324)/(-1.43); // Эмпирическая формула перевода ADC → °C
}

// Загрузка настроек из config.txt
static void load_config(void) {
    FRESULT res = f_open(&config_file, "config.txt", FA_READ);
    if (res != FR_OK) {
        // Создаем конфиг по умолчанию
        res = f_open(&config_file, "config.txt", FA_WRITE | FA_CREATE_ALWAYS);
        if (res == FR_OK) {
            char *default_cfg =
                "ALARM_INTERVAL=120\r\n"
                "MAG_ODR=50\r\n"
                "MAG_RANGE=2\r\n"
                "MAG_MODE=1\r\n";
            UINT bw;
            f_write(&config_file, default_cfg, strlen(default_cfg), &bw);
            f_close(&config_file);
            alarm_interval_sec = DEFAULT_ALARM_INTERVAL;
            qmc5883l_set_odr(MAG_ODR_50HZ);
            qmc5883l_set_range(MAG_RANGE_2GAUSS);
            qmc5883l_set_mode(MAG_CONTINUOUS);
        }
        return;
    }

    char line[64];
    while (f_gets(line, sizeof(line), &config_file)) {
        if (strncmp(line, "ALARM_INTERVAL=", 15) == 0) {
            alarm_interval_sec = atoi(line + 15);
        } else if (strncmp(line, "MAG_ODR=", 8) == 0) {
            uint8_t odr = atoi(line + 8);
            qmc5883l_set_odr(odr);
        } else if (strncmp(line, "MAG_RANGE=", 10) == 0) {
            uint8_t range = atoi(line + 10);
            qmc5883l_set_range(range);
        } else if (strncmp(line, "MAG_MODE=", 9) == 0) {
            uint8_t mode = atoi(line + 9);
            qmc5883l_set_mode(mode);
        }
    }
    f_close(&config_file);
}

// Сохранение конфигурации
static void save_config(void) {
    FRESULT res = f_open(&config_file, "config.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res == FR_OK) {
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
            "ALARM_INTERVAL=%lu\r\n"
            "MAG_ODR=%u\r\n"
            "MAG_RANGE=%u\r\n"
            "MAG_MODE=%u\r\n",
            alarm_interval_sec, qmc5883l_get_odr(), qmc5883l_get_range(), qmc5883l_get_mode()
        );
        UINT bw;
        f_write(&config_file, buf, len, &bw);
        f_close(&config_file);
    }
}

// Обработчик команд UART
static void process_uart_command(const char *cmd) {
    char cmd_upper[CMD_BUF_SIZE];
    strncpy(cmd_upper, cmd, CMD_BUF_SIZE);
    for (int i = 0; cmd_upper[i]; i++)
        if (cmd_upper[i] >= 'a' && cmd_upper[i] <= 'z')
            cmd_upper[i] -= 32;

    if (strcmp(cmd_upper, "TIME") == 0) {
        char buf[32];
        uint32_t t = pcf8563_get_unix_time();
        snprintf(buf, sizeof(buf), "%lu\r\n", t);
        uart_send_string(buf);
    } else if (strcmp(cmd_upper, "MEASURE") == 0) {
        log_measurement();
        uart_send_string("OK\r\n");
    } else if (strncmp(cmd_upper, "LOG ", 4) == 0) {
        uint32_t start_unix = 0;
        uint8_t num = 1;
        sscanf(cmd + 4, "%lu %hhu", &start_unix, &num);
        if (num > 10) num = 10;

        FRESULT res = f_open(&log_file, "log.txt", FA_READ);
        if (res == FR_OK) {
            char line[128];
            uint8_t sent = 0;
            while (f_gets(line, sizeof(line), &log_file)) {
                uint32_t log_time = 0;
                sscanf(line, "%lu", &log_time);
                if (log_time >= start_unix && sent < num) {
                    uart_send_string(line);
                    sent++;
                }
            }
            f_close(&log_file);
        } else {
            uart_send_string("ERROR: Cannot open log.txt\r\n");
        }
    } else if (strcmp(cmd_upper, "FORMAT_SD") == 0) {
        FRESULT res = f_mkfs("", 0, 0);
        if (res == FR_OK) {
            res = f_open(&log_file, "log.txt", FA_WRITE | FA_CREATE_ALWAYS);
            if (res == FR_OK) f_close(&log_file);
            res = f_open(&config_file, "config.txt", FA_WRITE | FA_CREATE_ALWAYS);
            if (res == FR_OK) f_close(&config_file);
            uart_send_string("OK\r\n");
        } else {
            uart_send_string("ERROR: Format failed\r\n");
        }
    } else if (strcmp(cmd_upper, "REBOOT") == 0) {
        uart_send_string("Rebooting...\r\n");
        _delay_ms(100);
        wdt_enable(WDTO_15MS);
        while(1); // Ждем перезагрузки
    } else if (strcmp(cmd_upper, "HELP") == 0) {
        uart_send_string(
            "Commands:\r\n"
            "TIME - get unix time\r\n"
            "MEASURE - log measurement\r\n"
            "LOG <unix> <num> - get logs\r\n"
            "FORMAT_SD - format SD\r\n"
            "REBOOT - reboot MCU\r\n"
            "HELP - show this help\r\n"
            "SET_INTERVAL <sec> - set log interval\r\n"
            "SET_MAG_ODR <10/50/100/200> - set magnetometer ODR\r\n"
            "SET_MAG_RANGE <2/8> - set magnetometer range\r\n"
            "SET_MAG_MODE <0/1> - set magnetometer mode\r\n"
            "GET_INTERVAL - get current interval\r\n"
            "GET_MAG_ODR - get magnetometer ODR\r\n"
            "GET_MAG_RANGE - get magnetometer range\r\n"
            "GET_MAG_MODE - get magnetometer mode\r\n"
        );
    } else if (strncmp(cmd_upper, "SET_INTERVAL ", 13) == 0) {
        uint32_t val = atoi(cmd + 13);
        if (val >= 10 && val <= 3600) {
            alarm_interval_sec = val;
            save_config();
            uart_send_string("OK\r\n");
        } else {
            uart_send_string("ERROR: Invalid interval\r\n");
        }
    } else if (strncmp(cmd_upper, "SET_MAG_ODR ", 12) == 0) {
        uint8_t odr = atoi(cmd + 12);
        if (qmc5883l_set_odr(odr)) {
            save_config();
            uart_send_string("OK\r\n");
        } else {
            uart_send_string("ERROR: Invalid ODR\r\n");
        }
    } else if (strncmp(cmd_upper, "SET_MAG_RANGE ", 14) == 0) {
        uint8_t range = atoi(cmd + 14);
        if (qmc5883l_set_range(range)) {
            save_config();
            uart_send_string("OK\r\n");
        } else {
            uart_send_string("ERROR: Invalid range\r\n");
        }
    } else if (strncmp(cmd_upper, "SET_MAG_MODE ", 13) == 0) {
        uint8_t mode = atoi(cmd + 13);
        if (qmc5883l_set_mode(mode)) {
            save_config();
            uart_send_string("OK\r\n");
        } else {
            uart_send_string("ERROR: Invalid mode\r\n");
        }
    } else if (strcmp(cmd_upper, "GET_INTERVAL") == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu\r\n", alarm_interval_sec);
        uart_send_string(buf);
    } else if (strcmp(cmd_upper, "GET_MAG_ODR") == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u\r\n", qmc5883l_get_odr());
        uart_send_string(buf);
    } else if (strcmp(cmd_upper, "GET_MAG_RANGE") == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u\r\n", qmc5883l_get_range());
        uart_send_string(buf);
    } else if (strcmp(cmd_upper, "GET_MAG_MODE") == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%u\r\n", qmc5883l_get_mode());
        uart_send_string(buf);
    } else {
        uart_send_string("ERROR: Unknown command\r\n");
    }
}

// Прерывание по приходу символа на приём (RX)
ISR(USART0_RX_vect) {
    char received_byte = UDR0;
    uart_rx_buffer[uart_rx_head++] = received_byte;
    uart_rx_head &= (UART_BUFFER_SIZE - 1); // Обновляем голову буфера
}

// Главная функция программы
int main(void) {
    cli(); // Отключаем глобальные прерывания на время начальной настройки

    // Инициализация UART
    uart_init(BAUD_RATE);

    // Инициализация I2C
    i2c_init();

    // Инициализация RTC
    pcf8563_init();

    // Инициализация магнитометра
    qmc5883l_init();

    // Монтируем SD-карту
    if (f_mount(&fs, "", 1) != FR_OK) {
        uart_send_string("ERROR: SD mount fail\r\n");
        while(1);
    }

    // Загрузка конфигурации
    load_config();

    // Настройка RTC будильника
    pcf8563_set_alarm_interval(alarm_interval_sec);

    // Включаем прерывания
    sei();

    // Основное тело программы
    while (1) {
        if (uart_available_bytes()) {
            char rx_byte = uart_get_char();
            // Обработать полученный символ
        }
        if (rtc_alarm_flag) {
            rtc_alarm_flag = false;
            log_measurement();
            pcf8563_set_alarm_interval(alarm_interval_sec);
        }
    }

    return 0;
}