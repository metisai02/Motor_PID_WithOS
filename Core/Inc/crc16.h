#ifndef __CRC16_H
#define __CRC16_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include <stdint.h>

uint16_t crc16_buff(const uint8_t *buf, int len);
uint16_t crc16_floating(uint8_t next, uint16_t seed);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CRC16_H */
