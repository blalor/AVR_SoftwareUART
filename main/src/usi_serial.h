/*
 * Based on AVR307
 */

#ifndef USI_SERIAL_H
#define USI_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#define DATA_BITS      8
#define START_BITS     1
#define STOP_BITS      1
#define PARITY_BITS    1

#define USI_COUNTER_MAX_COUNT 16

// @todo parameterize for use with other prescaler values
#define PCINT_STARTUP_DELAY 28
#define OCR_STARTUP_DELAY    8

#define USI_COUNTER_RECEIVE_SEED (USI_COUNTER_MAX_COUNT - DATA_BITS)
#define USI_COUNTER_PARITY_SEED  (USI_COUNTER_MAX_COUNT - PARITY_BITS)

// must match tested range in TEST(USISerialTests, BaudRateChecks).
typedef enum __baud_rate {
     BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
} BaudRate;

typedef struct __usi_ser_rx_regs {
    volatile uint8_t *pPORTB;
    volatile uint8_t *pPINB;
    volatile uint8_t *pDDRB;
    volatile uint8_t *pUSIBR;
    volatile uint8_t *pUSICR;
    volatile uint8_t *pUSISR;
    volatile uint8_t *pGIFR;
    volatile uint8_t *pGIMSK;
    volatile uint8_t *pPCMSK;
} USISerialRxRegisters;

void usi_serial_receiver_init(
    const USISerialRxRegisters *reg,
    void (*received_byte_handler)(uint8_t),
    const BaudRate baud_rate,
    const bool enable_even_parity
);

#endif
