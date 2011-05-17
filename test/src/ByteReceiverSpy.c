#include "ByteReceiverSpy.h"

static uint8_t received_byte;
static uint8_t invocation_count;

void brs_init() {
    received_byte = 0;
    invocation_count = 0;
}

void brs_receive_byte(uint8_t b) {
    received_byte = b;
    invocation_count += 1;
}

uint8_t brs_get_received_byte() {
    return received_byte;
}

uint8_t brs_get_invocation_count() {
    return invocation_count;
}
