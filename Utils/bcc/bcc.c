#include <stdint.h>

/**
 * @brief bcc (Block Check Character)异或校验
 *
 * @param start_byte 用于校验的数据
 * @param len 用于校验的长度
 * @return uint8_t bcc校验值
 */
uint8_t bcc8(uint8_t *start_byte, uint16_t len) {
    uint8_t i = 0;
    uint8_t bcc = 0;
    for (i = 0; i < len; i++) {
        bcc ^= start_byte[i];
    }
    return bcc;
}