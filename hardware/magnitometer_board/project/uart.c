// uart.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>

// Буфер приёма
volatile char uart_rx_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_rx_head = 0;
volatile uint8_t uart_rx_tail = 0;

// Количество элементов в кольцевом буфере
inline uint8_t uart_available_bytes() {
    return (uart_rx_head - uart_rx_tail) & (UART_BUFFER_SIZE - 1);
}

// Инициализация UART
void uart_init(uint16_t baud_rate) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1<<U2X0);
#else
    UCSR0A &= ~(1<<U2X0);
#endif
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); // Enable RX interrupt
    UCSR0C = (1<<USBS0)|(3<<UCSZ00); // Asynchronous, no parity, 1 stop bit, 8-bit character size
}

// Отправка символа по UART
void uart_send_char(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0); // Wait until buffer is empty
    UDR0 = c;
}

// Отправка строки по UART
void uart_send_string(const char* str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

// Прерывание по приходу символа на приём (RX)
ISR(USART0_RX_vect) {
    char received_byte = UDR0;
    uart_rx_buffer[uart_rx_head++] = received_byte;
    uart_rx_head &= (UART_BUFFER_SIZE - 1); // Обновляем позицию головы буфера
}

// Функция для чтения символа из буфера
char uart_get_char() {
    char ret_val;
    if (uart_available_bytes()) {
        ret_val = uart_rx_buffer[uart_rx_tail++];
        uart_rx_tail &= (UART_BUFFER_SIZE - 1); // Обновляем хвост буфера
        return ret_val;
    }
    return -1; // Нет доступных данных
}