/*
 * Based on AVR307
 */

#ifndef USI_SERIAL_H
#define USI_SERIAL_H

#include <stdint.h>

/*
F_CPU = 8000000 Hz => 0.0125 µS / cycle

F_CPU/8 = 1000000 Hz => 1 µS / cycle

baud rate = 9600
9600 bits/1 second
104.16667 µS/1 bit
52.083 µS/0.5 bit

cycles/bit = F_CPU/(baud * prescale)

              Timer Seed   % err    
F_CPU (kHz)   ====== 8000 ======     
prescale                             
   1                --       --      
   8           104.167    0.160      
  64            13.021    0.161      
 256             3.255    7.834      
1024                --       --      
*/

#define BAUD_RATE   9600
#define DATA_BITS      8
#define START_BITS     1
#define STOP_BITS      1
#define PARITY_BITS    1

#define USI_COUNTER_MAX_COUNT 16 // @todo verify; 0-15??

#define TIMER0_SEED (( F_CPU / BAUD_RATE) / 8)

// @todo parameterize for use with other prescaler values
#define PCINT_STARTUP_DELAY 28
#define OCR_STARTUP_DELAY    8

#define INITIAL_TIMER0_SEED (( TIMER0_SEED * 3 ) / 2)

#define USI_COUNTER_RECEIVE_SEED (USI_COUNTER_MAX_COUNT - DATA_BITS)
#define USI_COUNTER_PARITY_SEED  (USI_COUNTER_MAX_COUNT - PARITY_BITS)

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

void usi_serial_receiver_init(const USISerialRxRegisters *reg, void (*received_byte_handler)(uint8_t));

#endif
