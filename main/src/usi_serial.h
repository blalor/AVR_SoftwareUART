/*
 * Based on AVR307
 *
 * NOTE:
 *    DO requires external pull-up (@todo verify)
 */

#ifndef USI_SERIAL_H
#define USI_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

// @todo parameterize for use with other prescaler values
#define PCINT_STARTUP_DELAY 28

// mainly for reference for interested parties; 8 data bits and
// (optionally) 1 parity bit are all that this driver can handle.
#define DATA_BITS   8
#define PARITY_BITS 1

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
    volatile uint8_t *pUSIDR;
    volatile uint8_t *pUSISR;
    volatile uint8_t *pGIFR;
    volatile uint8_t *pGIMSK;
    volatile uint8_t *pPCMSK;
} USISerialRegisters;

/*
 * Initialize USI Serial receiver.
 *
 * @param reg register config struct
 * @param received_byte_handler pointer to handler of received bytes
 * @param baud_rate the baud rate to operate at
 * @param enable_even_parity true if using even parity
 */
void usi_serial_init(
    const USISerialRegisters *reg,
    void (*received_byte_handler)(uint8_t),
    const BaudRate baud_rate,
    const bool enable_even_parity
);

/*
 * Transmit a byte.
 *
 * @param b the byte to transmit
 */
uint8_t usi_tx_byte(const uint8_t b);

#endif
