#ifndef __CRC_H
#define __CRC_H

#include <stdint.h>

uint16_t calc_crc16(uint8_t *start_byte, uint16_t len);
uint8_t calc_crc8(uint8_t *start_byte, uint8_t len);

#endif /* __CRC_H */