#ifndef CONF_USART_SERIAL_H
#define CONF_USART_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* USART serial configuration options. */
#define DBG_USART                       (&AVR32_USART0)
#define USART_SERIAL_EXAMPLE_BAUDRATE   (115200)
#define USART_SERIAL_CHAR_LENGTH        (8)
#define USART_SERIAL_PARITY             (USART_NO_PARITY)
#define USART_SERIAL_STOP_BIT           (USART_1_STOPBIT)

#ifdef __cplusplus
}
#endif

#endif /* CONF_USART_SERIAL_H */
