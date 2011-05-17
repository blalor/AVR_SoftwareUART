#ifndef BYTE_RECEIVER_SPY_H
#define BYTE_RECEIVER_SPY_H

#include <stdint.h>

void brs_init(void);

void brs_receive_byte(uint8_t b);
uint8_t brs_get_received_byte(void);
uint8_t brs_get_invocation_count(void);

#endif
