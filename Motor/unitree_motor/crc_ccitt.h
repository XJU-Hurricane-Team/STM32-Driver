#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include <stdint.h>
#include <stdlib.h>


uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);


#endif
