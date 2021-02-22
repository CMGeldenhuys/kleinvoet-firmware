/**
 *
 * @file serial.h
 * @author CM Geldenhuys
 * @date 30 Jul. 2020
 *
 * @brief Timestamping metadata on a per sample basis using a timer module to
 * track samples
 *
 * Timer module is driven by the ADC's frame clock to keep track of the current
 * frame. This information is combined with time code information sent by the
 * GPS module. THis allows one to timestamp on a per frame basis. Since the GPS
 * needs time to compute internally compute the latest timing information; a
 * sample is first marked and then stamped. The abstraction also offers an API
 * to stamp arbitrary samples with meta data.
 *
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#ifndef SERIAL_MAX_DELAY
#define SERIAL_MAX_DELAY HAL_MAX_DELAY
#endif

#define SERIAL_CRLF "\r\n"
#define SERIAL_BS "\b"

#ifndef SERIAL_EOL
#define SERIAL_EOL SERIAL_CRLF
#endif

#ifndef SERIAL_BUF_LEN
#define SERIAL_BUF_LEN 256
#endif

#ifndef SERIAL_RX_LEN
#define SERIAL_RX_LEN SERIAL_BUF_LEN
#endif

#ifndef SERIAL_TX_LEN
#define SERIAL_TX_LEN SERIAL_BUF_LEN
#endif

#if !defined(SERIAL_RX_DMA) || !defined(SERIAL_RX_BLOCKING) || !defined(SERIAL_RX_INTERRUPT)
#define SERIAL_RX_DMA
#endif

#if !defined(SERIAL_TX_DMA) || !defined(SERIAL_TX_BLOCKING) || !defined(SERIAL_TX_INTERRUPT)
#define SERIAL_TX_BLOCKING
#endif

typedef struct {
    struct {
        UART_HandleTypeDef *uart;
    } config_;

    struct {
        uint8_t buf_[SERIAL_RX_LEN];
        size_t  idx;
    } rx;

    struct {
        uint8_t buf_[SERIAL_TX_LEN];
        size_t  idx;
    } tx;

} Serial_t;

/**
 * @brief Wrap STM HAL interface with own serial abstraction
 *
 * @param [in] self Serial object
 * @param [in] uart HAL interface
 *
 * @return Returns a value greater than 0 if successful
 */
int Serial_wrap (Serial_t *self, UART_HandleTypeDef *uart);

/**
 * @brief Returns amount of bytes available to read on serial
 *
 * @param [in] self Serial interface
 *
 * @return Returns number of bytes available on serial port
 */
size_t Serial_available (Serial_t *self);

/**
 * @brief Reads byte from serial port
 *
 * @param [in] self Serial Interface
 *
 * @return Returns byte read from serial
 */
uint8_t Serial_read (Serial_t *self);

/**
 * @brief Writes data to serial port
 *
 * @param [in] self Serial Interface
 * @param [in] buf Data to be written to serial interface
 * @param [in] len Size of data buffer to be written
 *
 * @return Returns a value greater than 0 if successful
 */
int Serial_write (Serial_t *self, const uint8_t *buf, size_t len);

/**
 * @brief Writes a string to serial interface with newline ending
 *
 * @param [in] self Serial Interface
 * @param [in] str String to be written
 *
 * @see Serial_write (Serial_t *, const uint8_t *, size_t)
 * @see Serial_print (Serial_t *, const char *)
 *
 * @return Returns a value greater than 0 if successful
 */
int Serial_println (Serial_t *self, const char *str);

/**
 * @brief Writes a string to serial interface
 *
 * @param [in] self Serial Interface
 * @param [in] str String to be written
 *
 * @see Serial_write (Serial_t *, const uint8_t *, size_t)
 *
 * @return Returns a value greater than 0 if successful
 */
int Serial_print (Serial_t *self, const char *str);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H_ */
