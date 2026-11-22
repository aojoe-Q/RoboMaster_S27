#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#include <stdint.h>
#include <stdbool.h>

/*
 * referee_usart_task.c carries its own CRC implementation. It calls the
 * CRC16 checker with a capital "CRC" spelling while the local definition uses
 * lowercase; map the spelling so the APP source stays untouched.
 */
bool verify_crc8_check_sum(const uint8_t *data, uint32_t length);
bool verify_crc16_check_sum(const uint8_t *data, uint32_t length);
#define verify_CRC16_check_sum verify_crc16_check_sum

#endif /* CRC8_CRC16_H */
