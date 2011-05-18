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
static const float bit_period = 1e6/_BAUD_RATE;

static const USISerialRxRegisters usiRegs = {
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

TEST_GROUP(USISerialTXTests) {
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
        usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, false);
    }
};

TEST(USISerialTXTests, Initialization) {
    // when not transmitting, leave TX/DO/PB1 as an *input* with the internal
    // pull-up enabled
    BYTES_EQUAL(B11111100, virtualDDRB);  // configure PB0, PB1 as inputs
    BYTES_EQUAL(B00000011, virtualPORTB); // internal pull-ups enabled for PB0, PB1
}

TEST(USISerialTXTests, TransmitByte) {
    // reverse byte
    // wait for rx-in-progress to complete
    
    // initialize for TX:
    //     set initial TX state flag
    //  X  disable pin-change interrupt (to disable RX)
    //  X  load USIDR with 0xff
    //  X  fire USI overflow on next clock tick
    //  X  set USI 3-wire mode
    //  X  set USI clock source (timer0 comp)
    //  X  enable USI overflow interrupt
    //  X  clear timer0 value
    //  X  set ocra to timer0_seed
    //  X  start timer0

    virtualPCMSK = 0xff;
    virtualUSIDR = 0;
    virtualUSICR = B10101011; // inverse of expected result
    virtualUSISR = 0;
    virtualTCNT0 = 0xff;
    virtualGTCCR = 0xff;
    virtualDDRB = 0;
    
    // 'e'
    //           B01100101, 101, 0x65
    // reversed: B10100110, 166, 0xA6
    usi_tx_byte('e');
    
    BYTES_EQUAL(B11111110, virtualPCMSK); // PCINT0 disabled
    BYTES_EQUAL(B00000010, virtualDDRB);  // PB1 configured as output
    BYTES_EQUAL(0xff,      virtualUSIDR); // USIDR set
    BYTES_EQUAL(0xff,      virtualUSISR); // flags cleared, overflow on next tick
    
    // USI 3-wire mode enabled, clock source set, overflow interrupt enabled
    BYTES_EQUAL(B01010100, virtualUSICR);
    
    BYTES_EQUAL(0,         virtualTCNT0); // timer0 cleared
    
    // check Timer0 configured to compare with OCR0A at the bit period
    DOUBLES_EQUAL(bit_period, virtualOCR0A, bit_period*0.02 /* 2% */);
    
    BYTES_EQUAL(0, virtualGTCCR >> 7); // confirm timer0 started
    
    // -- ok, now the first timer tick and overflow; first half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;

    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  1 (line idling high)
    //  0 (start bit)
    //  101001 (bits 0..5 of the letter 'e')
    BYTES_EQUAL(B10101001, virtualUSIDR);
    
    BYTES_EQUAL(B11111011, virtualUSISR); // flags cleared, overflow after 5 bits
    
    // -- now the 2nd tick/overflow; 2nd half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;
    
    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  00110 (bits 3..7 of the letter 'e')
    //  1 (stop bit)
    //  11 (padding)
    BYTES_EQUAL(B00110111, virtualUSIDR);
    
    // 2nd overflow doesn't actually need to send the stop bit
    // the line idles high, and the first bit sent when transmitting a byte
    // is a 1
    BYTES_EQUAL(B11111011, virtualUSISR); // flags cleared, overflow after 5 bits
    
    // -- the last tick/overflow; turn off USI, reset to idle state
    virtualPCMSK = 0;
    virtualDDRB = 0xff;
    virtualPORTB = 0;
    virtualUSICR = 0xff;
    
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(0,         virtualUSICR); // USI disabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 enabled
    BYTES_EQUAL(B11111101, virtualDDRB);  // PB1 configured as input
    BYTES_EQUAL(B00000010, virtualPORTB); // PB1 internal pull-up enabled
}

TEST(USISerialTXTests, TransmitByteWithParity) {
    usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, true);
    
    // 'e'
    //           B01100101, 101, 0x65
    // reversed: B10100110, 166, 0xA6
    usi_tx_byte('e');
    
    // -- ok, now the first timer tick and overflow; first half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;

    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  1 (line idling high)
    //  0 (start bit)
    //  101001 (bits 0..5 of the letter 'e')
    BYTES_EQUAL(B10101001, virtualUSIDR);
    
    BYTES_EQUAL(B11111011, virtualUSISR); // flags cleared, overflow after 5 bits
    
    // -- now the 2nd tick/overflow; 2nd half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;
    
    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  00110 (bits 3..7 of the letter 'e')
    //  0 (parity bit; even number of 1s in 'e')
    //  1 (stop bit)
    //  1 (padding)
    BYTES_EQUAL(B00110011, virtualUSIDR);
    
    // 2nd overflow doesn't actually need to send the stop bit
    // the line idles high, and the first bit sent when transmitting a byte
    // is a 1
    BYTES_EQUAL(B11111010, virtualUSISR); // flags cleared, overflow after 6 bits
    
    // -- the last tick/overflow; turn off USI, reset to idle state
    virtualPCMSK = 0;
    virtualDDRB = 0xff;
    virtualPORTB = 0;
    virtualUSICR = 0xff;
    
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(0,         virtualUSICR); // USI disabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 enabled
    BYTES_EQUAL(B11111101, virtualDDRB);  // PB1 configured as input
    BYTES_EQUAL(B00000010, virtualPORTB); // PB1 internal pull-up enabled
}

TEST(USISerialTXTests, TransmitByteWithParityOddOnes) {
    usi_serial_init(&usiRegs, &brs_receive_byte, BAUD_9600, true);
    
    // 'g'
    //           B01100111, 103, 0x67
    // reversed: B11100110, 230, 0xE6
    usi_tx_byte('g');
    
    // -- ok, now the first timer tick and overflow; first half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;

    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  1 (line idling high)
    //  0 (start bit)
    //  111001 (bits 0..5 of the letter 'g')
    BYTES_EQUAL(B10111001, virtualUSIDR);
    
    BYTES_EQUAL(B11111011, virtualUSISR); // flags cleared, overflow after 5 bits
    
    // -- now the 2nd tick/overflow; 2nd half-frame
    virtualUSIDR = 0;
    virtualUSISR = 0;
    
    ISR_USI_OVF_vect();
    
    // USIDR should have:
    //  00110 (bits 3..7 of the letter 'g')
    //  1 (parity bit; odd number of 1s in 'g')
    //  1 (stop bit)
    //  1 (padding)
    BYTES_EQUAL(B00110111, virtualUSIDR);
    
    // 2nd overflow doesn't actually need to send the stop bit
    // the line idles high, and the first bit sent when transmitting a byte
    // is a 1
    BYTES_EQUAL(B11111010, virtualUSISR); // flags cleared, overflow after 6 bits
    
    // -- the last tick/overflow; turn off USI, reset to idle state
    virtualPCMSK = 0;
    virtualDDRB = 0xff;
    virtualPORTB = 0;
    virtualUSICR = 0xff;
    
    ISR_USI_OVF_vect();
    
    BYTES_EQUAL(0,         virtualUSICR); // USI disabled
    BYTES_EQUAL(B00000001, virtualPCMSK); // PCINT0 enabled
    BYTES_EQUAL(B11111101, virtualDDRB);  // PB1 configured as input
    BYTES_EQUAL(B00000010, virtualPORTB); // PB1 internal pull-up enabled
}
