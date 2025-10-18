// uart.h
#ifndef UART_H_
#define UART_H_

#include <stdint.h>

// Размер буфера для приёма данных
#define UART_BUFFER_SIZE 128

// Определение глобальной переменной для буфера приёма
extern volatile char uart_rx_buffer[UART_BUFFER_SIZE];
extern volatile uint8_t uart_rx_head;
extern volatile uint8_t uart_rx_tail;

// Доступные функции UART
void uart_init(uint16_t baud_rate);
void uart_send_char(char c);
void uart_send_string(const char* str);
char uart_get_char(void);
uint8_t uart_available_bytes(void);

#endif /* UART_H_ */