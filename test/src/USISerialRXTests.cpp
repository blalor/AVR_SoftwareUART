/*
    well, this ought to be interesting, since this is normally completely
    in hardware…
*/

extern "C" {
    #include <avr/io.h>
    
    #include "8bit_binary.h"
    #include "usi_serial.h"
    #include "8bit_tiny_timer0.h"

    #include "ByteReceiverSpy.h"
    
    void ISR_PCINT0_vect(void);
    void ISR_TIMER0_COMPA_vect(void);
    void ISR_USI_OVF_vect(void);
}

#include <stdint.h>
#include "CppUTest/TestHarness.h"

static const float _BAUD_RATE = (float) BAUD_9600;

static const USISerialRegisters usiRegs = {
    &virtualPORTB,
    &virtualPINB,
    &virtualDDRB,
    &virtualUSIBR,
    &virtualUSICR,
    &virtualUSIDR,
    &virtualUSISR,
    &virtualGIFR,
    &virtualGIMSK,
    &virtualPCMSK,
};

static const Timer0Registers timer0Regs = {
    &virtualGTCCR,
    &virtualTCCR0A,
    &virtualTCCR0B,
    &virtualOCR0A,
    &virtualTIMSK,
    &virtualTIFR,
    &virtualTCNT0,
};

TEST_GROUP(USISerialRXTests) {
    void setup() {
        virtualPORTB = 0;
        virtualDDRB = 0xff;
        virtualUSIBR = 0;
        virtualUSICR = 0xff;
        virtualUSISR = 0xff;
        virtualGIFR = 0;
        virtualGIMSK = 0;
        virtualPCMSK = 0;

        virtualGTCCR = 0;
        virtualTCCR0A = 0;
        virtualTCCR0B = 0;
        virtualOCR0A = 0;
        virtualTIMSK = 0;
        virtualTIFR = 0;
        virtualTCNT0 = 0;
        
        // init byte receiver spy
        brs_init();
        
        // must initialize Timer0 first
        timer0_init(&timer0Regs, TIMER0_PRESCALE_8);
        usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, true);
    }
};

TEST(USISerialRXTests, ClockSpeed) {
    // sanity check; need to change if CPU clock changes, but the timer 
    // config is hard-coded.
    CHECK(F_CPU == 8000000);
}

TEST(USISerialRXTests, Initialization) {
    virtualPORTB = 0x0;
    virtualDDRB = 0xff;
    virtualUSICR = 0xff;
    virtualGIMSK = 0;
    virtualPCMSK = 0;
    virtualTCCR0B = 0xff;
    
    usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, true);

    // see comment in usi_serial_init re: reasoning for DO as input
    BYTES_EQUAL(B00000011, virtualPORTB); // DI, DO pull-ups enabled
    BYTES_EQUAL(B11111100, virtualDDRB);  // DI, DO configured for input
    BYTES_EQUAL(B00000000, virtualUSICR); // USI disabled
    BYTES_EQUAL(B00100000, virtualGIMSK); // PCINTs enabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 enabled
    
    // verify timer0 config
    BYTES_EQUAL(B00000010, virtualTCCR0A); // CTC enabled
    BYTES_EQUAL(B11110000, virtualTCCR0B); // CTC enabled, timer stopped, no prescaler
    BYTES_EQUAL(0,         virtualTIMSK);  // OCR0A comp match interrupt disabled
}

TEST(USISerialRXTests, HandleStartBit) {
    /*
    the DI line idles high; need to trigger the pin-change interrupt, 
    emulating a change from high to low.
    */
    
    virtualPINB  = B11111110;
    virtualTCNT0 = 99;
    virtualGTCCR = 0xff;
    virtualPCMSK = 0xff;
    
    // fire the PCI
    ISR_PCINT0_vect();
    
    // ----- check Timer0 started
    BYTES_EQUAL(0,         virtualTCNT0);  // timer value set to 0
    BYTES_EQUAL(B00000010, virtualTCCR0B); // prescaler; cpu/8
    BYTES_EQUAL(B01111111, virtualGTCCR);  // TSM bit cleared
    
    BYTES_EQUAL(B00010000, virtualTIMSK);  // OCR0A compare interrupt enabled
    
    // check Timer0 configured to compare with OCR0A at 1.5 times the bit 
    // period, plus the interrupt latency
    float bit_period = 1e6/_BAUD_RATE;
    DOUBLES_EQUAL(
        (bit_period * 1.5) - PCINT_STARTUP_DELAY,
        virtualOCR0A,
        ((bit_period * 1.5) - PCINT_STARTUP_DELAY)*0.02 // 2%
    );
    
    // ----- check USI config
    
    // clear USI status interrupt flags; don't care about bit 4 (USIDC)
    BYTES_EQUAL(0x0f, virtualUSISR >> 4); 
    BYTES_EQUAL(8, 0x0f & virtualUSISR);

    // USI overflow interrupt, 3-wire mode, Timer0 compare as clock source
    BYTES_EQUAL(B01010100, virtualUSICR);
    
    BYTES_EQUAL(B11111110, virtualPCMSK); // PCI disabled
}

/*
 * Checks the allowable range of baud rates.
 *
 * subset of HandleStartBit above
 */
TEST(USISerialRXTests, BaudRateChecks) {
    BaudRate baud_rates[] = {
        BAUD_9600,
        BAUD_19200,
        // BAUD_28800, fails DOUBLES_EQUAL
        BAUD_38400,
    };
    
    for (uint8_t br_ind = 0; br_ind < (sizeof(baud_rates)/sizeof(baud_rates[0])); br_ind++) {
        BaudRate baud_rate = baud_rates[br_ind];
        
        virtualTCCR0B = 0;

        usi_serial_init(&usiRegs, &brs_receive_byte, baud_rate, true);

        /*
        the DI line idles high; need to trigger the pin-change interrupt, 
        emulating a change from high to low.
        */

        virtualPINB  = B11111110;
        virtualTCNT0 = 99;
        virtualGTCCR = 0xff;
        virtualPCMSK = 0xff;

        // fire the PCI
        ISR_PCINT0_vect();

        // ----- check Timer0 started
        BYTES_EQUAL(0,         virtualTCNT0);  // timer value set to 0
        BYTES_EQUAL(B00000010, virtualTCCR0B); // prescaler; cpu/8
        BYTES_EQUAL(B01111111, virtualGTCCR);  // TSM bit cleared

        BYTES_EQUAL(B00010000, virtualTIMSK);  // OCR0A compare interrupt enabled

        // check Timer0 configured to compare with OCR0A at 1.5 times the bit 
        // period, plus the interrupt latency
        float bit_period = (1e6/((float) baud_rate));
        DOUBLES_EQUAL(
            (bit_period * 1.5) - PCINT_STARTUP_DELAY,
            virtualOCR0A,
            ((bit_period * 1.5) - PCINT_STARTUP_DELAY)*0.02 // 2%
        );
    }
}

TEST(USISerialRXTests, HandleByteReceivedPlusParityBit) {
    ISR_PCINT0_vect();
    
    // fire first compare interrupt, assert new ocra value, confirm ocra 
    // interrupt disabled
    virtualOCR0A = 0;
    virtualTIMSK = 0xff;
    
    ISR_TIMER0_COMPA_vect();
    
    // parameterize prescale if necessary
    BYTES_EQUAL((uint8_t)(F_CPU/_BAUD_RATE/8), virtualOCR0A);
    BYTES_EQUAL(B11101111,   virtualTIMSK); // OCR0A compare interrupt disabled
    
    // assume USI is configured correctly and has received 8 bits, in reverse
    // order. we're going to pretend an 'a' has been sent
                   
    virtualUSIBR = B10000110; // 'a' reversed
    virtualTCCR0B = 0xff;
    virtualPCMSK = 0;
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(1, brs_get_invocation_count());
    BYTES_EQUAL('a', brs_get_received_byte());
    
    // ----- check config for/before "consuming" parity bit
    BYTES_EQUAL(0xff, virtualTCCR0B); // no change, yet
    BYTES_EQUAL(0,    virtualPCMSK); // no change, yet
    
    // clear USI status interrupt flags; don't care about bit 4 (USIDC)
    BYTES_EQUAL(0x0f, virtualUSISR >> 4); 
    BYTES_EQUAL(15,   0x0f & virtualUSISR);  // just one bit left to consume
    
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(1, brs_get_invocation_count()); // still called just once
    
    BYTES_EQUAL(B11111000, virtualTCCR0B); // timer0 prescaler cleared
    BYTES_EQUAL(0,         virtualUSICR); // USI disabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 re-enabled
    // @todo confirm other register settings
}

TEST(USISerialRXTests, HandleByteReceivedNoParity) {
    usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, false);
    
    // signal start bit has … uh … started
    ISR_PCINT0_vect();
    
    // fire first compare interrupt, assert new ocra value, confirm ocra 
    // interrupt disabled
    virtualOCR0A = 0;
    virtualTIMSK = 0xff;
    
    ISR_TIMER0_COMPA_vect();
    
    // parameterize prescale if necessary
    BYTES_EQUAL((uint8_t)(F_CPU/_BAUD_RATE/8), virtualOCR0A);
    BYTES_EQUAL(B11101111, virtualTIMSK); // OCR0A compare interrupt disabled
    
    // assume USI is configured correctly and has received 8 bits, in reverse
    // order. we're going to pretend an 'a' has been sent
                   
    virtualUSIBR = B10000110; // 'a' reversed
    virtualTCCR0B = 0xff;
    virtualPCMSK = 0;
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(1, brs_get_invocation_count());
    BYTES_EQUAL('a', brs_get_received_byte());
    
    // ----- check config; no parity bit. timer and USI should be disabled
    BYTES_EQUAL(B11111000, virtualTCCR0B); // timer0 prescaler cleared
    BYTES_EQUAL(0,         virtualUSICR); // USI disabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 re-enabled
    // @todo confirm other register settings
}
