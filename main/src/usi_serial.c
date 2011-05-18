#include <avr/interrupt.h>
#include <util/parity.h>

#include "8bit_tiny_timer0.h"

#include "usi_serial.h"
#define USI_COUNTER_MAX_COUNT 16
#define HALF_FRAME 5

typedef enum __usi_rx_state {
    USIRX_STATE_IDLE,
    USIRX_STATE_RECEIVING,
    USIRX_STATE_WAITING_FOR_PARITY_BIT,
} USIRxState;

typedef enum __usi_tx_state {
    USITX_STATE_IDLE,
    USITX_STATE_READY_FOR_FIRST_HALF_FRAME,
    USITX_STATE_READY_FOR_SECOND_HALF_FRAME,
    USITX_STATE_COMPLETE,
} USITxState;

static const USISerialRegisters *reg;
static void (*received_byte_handler)(uint8_t);

static bool even_parity_enabled;
static uint8_t timer0_seed;
static uint8_t initial_timer0_seed;

static uint8_t pending_tx_byte;
static volatile USIRxState rxState;
static volatile USITxState txState;

// not part of the public interface
static void usi_handle_ocra_reload(void);

// Reverses the order of bits in a byte.
// i.e. MSB is swapped with LSB, etc.
static inline uint8_t reverse_bits(const uint8_t to_swap) {
    uint8_t x = to_swap;
    
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    
    return x;    
}

void usi_serial_init(const USISerialRegisters *_reg,
                     void (*_handler)(uint8_t),
                     const BaudRate baud_rate,
                     const bool enable_even_parity)
{
    reg = _reg;
    received_byte_handler = _handler;
    even_parity_enabled = enable_even_parity;
    
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
    
    // cycles/bit = F_CPU/(baud * prescale)
    timer0_seed = (uint8_t)(( F_CPU / ((float) baud_rate)) / 8);
    
    // 1.5 times timer0_seed
    initial_timer0_seed =
        ((uint8_t)(( timer0_seed * 3 ) / 2)) - PCINT_STARTUP_DELAY;

    rxState = USIRX_STATE_IDLE;
    txState = USITX_STATE_IDLE;
    
    // yes, we're configuring TX as *input* as well, so that the internal
    // pull-up keeps the line high.  This will be overridden by the USI when
    // switching to 3-wire mode
    *reg->pDDRB  &= ~(_BV(PB0) | _BV(PB1)); // set RX *and* TX pins as inputs
    *reg->pPORTB |= _BV(PB0) | _BV(PB1);    // enable pull-up on RX *and* TX pins
    
    *reg->pUSICR  = 0;           // disable USI
    
    *reg->pGIFR  &= ~_BV(PCIF);  // clear PCI flag, just because
    *reg->pGIMSK |= _BV(PCIE);   // enable PCIs
    *reg->pPCMSK |= _BV(PCINT0); // enable PCINT0
    
    // prepare timer0; not started until PCINT0 fires
    timer0_set_ocra_interrupt_handler(&usi_handle_ocra_reload);
    timer0_enable_ctc();
    timer0_stop();
}

int8_t usi_tx_byte(const uint8_t b) {
    // wait for tx- or rx-in-progress to complete
    while ((rxState != USIRX_STATE_IDLE) && (txState != USITX_STATE_IDLE));

    txState = USITX_STATE_READY_FOR_FIRST_HALF_FRAME;
    
    // reverse byte
    pending_tx_byte = reverse_bits(b);
    
    *reg->pPCMSK &= ~_BV(PCINT0); // disable PCINT0
    *reg->pDDRB |= _BV(PB1);      // configure PB1 as output
    *reg->pUSIDR = 0xff;          // drive line high until data provided
    
    // enable USI overflow interrupt
    // set USI 3-wire mode
    // set USI clock source to timer0 compare
    *reg->pUSICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);
    
    *reg->pUSISR = 0xF0 | // clear flags
                   0x0F;  // timer to just-about-to-overflow
    
    timer0_set_counter(0);
    timer0_set_ocra(timer0_seed);
    timer0_start();
    
    return 0;
}

// @todo refactor this so that the PCINT0 ISR is configured in main()
ISR(PCINT0_vect) {
    if ((*reg->pPINB & _BV(PB0)) == 0) {
        // PB0 is low; start bit received
        // do the time-critical stuff first
        
        // configure the timer to fire the OCR0A compare interrupt in the
        // middle of the first data bit
        timer0_set_counter(0);
        timer0_set_ocra(initial_timer0_seed);
        timer0_enable_ocra_interrupt();
        timer0_start();
        
        // ----- configure the USI
        // clear interrupt flags, prepare for data bit count
        // overflow should occur when all data bits are received
        *reg->pUSISR = 0xF0 | (USI_COUNTER_MAX_COUNT - DATA_BITS);
        
        // enable overflow interrupt, set 3-wire mode, clock from timer0 comp
        *reg->pUSICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);
        
        // ----- time-critical stuff done
        *reg->pPCMSK &= ~_BV(PCINT0); // disable PCINT0
        
        rxState = USIRX_STATE_RECEIVING;
    }
}

static void usi_handle_ocra_reload() {
    // set the OCR0A match to the bit duration and disable the OCR0A compare
    // interrupt; with CTC mode, the timer's reset, and the OCR0A match clocks
    // the USI in hardware
    
    timer0_set_ocra(timer0_seed);
    timer0_disable_ocra_interrupt();
}

// USI overflow interrupt.  Configured to occur when the desired number of bits
// have been shifted in (in reverse order!)
ISR(USI_OVF_vect) {
    if (txState != USITX_STATE_IDLE) {
        if (txState == USITX_STATE_READY_FOR_FIRST_HALF_FRAME) {
            // load USIDR with a 1, the start bit, and the first 5 bits of the byte
            *reg->pUSIDR = 0x80 | (pending_tx_byte >> 2);

            // set up next overflow to reload USIDR with the remaining half of the byte
            *reg->pUSISR = 0xf0 | (USI_COUNTER_MAX_COUNT - HALF_FRAME);
            
            txState = USITX_STATE_READY_FOR_SECOND_HALF_FRAME;
        }
        else if (txState == USITX_STATE_READY_FOR_SECOND_HALF_FRAME) {
            // load USIDR with the last 5 bits of the byte and pad with 1s
            *reg->pUSIDR = (pending_tx_byte << 3) | 0x07;
            
            if (even_parity_enabled) {
                if (parity_even_bit(pending_tx_byte)) {
                    *reg->pUSIDR |= _BV(2);
                }
                else {
                    *reg->pUSIDR &= ~_BV(2);
                }

                // set up next overflow to shut down USI; additional tick for
                // parity bit
                *reg->pUSISR = 0xf0 | (USI_COUNTER_MAX_COUNT - (HALF_FRAME + 1));
            }
            else {
                // set up next overflow to shut down USI
                *reg->pUSISR = 0xf0 | (USI_COUNTER_MAX_COUNT - HALF_FRAME);
            }

            txState = USITX_STATE_COMPLETE;
        }
        else /* USITX_STATE_COMPLETE */ {
            *reg->pUSICR = 0;            // disable USI
            *reg->pPCMSK |= _BV(PCINT0); // re-enable PCINT
            *reg->pDDRB &= ~_BV(PB1);    // PB1 as input
            *reg->pPORTB |= _BV(PB1);    // PB1 internal pull-up enabled
            
            txState = USITX_STATE_IDLE;
        }
    }
    else {
        if (rxState == USIRX_STATE_RECEIVING) {
            // WARNING! this is being called in an ISR and MUST be very fast!
            received_byte_handler(reverse_bits(*reg->pUSIBR));
        }

        if (even_parity_enabled && (rxState == USIRX_STATE_RECEIVING)) {
            // clear interrupt flags, prepare for parity bit count
            // overflow should occur when all parity bits are received
            *reg->pUSISR = 0xF0 | (USI_COUNTER_MAX_COUNT - PARITY_BITS);
            rxState = USIRX_STATE_WAITING_FOR_PARITY_BIT;
        }
        else {
            // disable timer
            timer0_stop();

            *reg->pUSICR = 0;            // disable USI
            *reg->pPCMSK |= _BV(PCINT0); // re-enable PCINT

            rxState = USIRX_STATE_IDLE;
        }
    }
}
