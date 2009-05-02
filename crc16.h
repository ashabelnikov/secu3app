
#ifndef _CRC16_H_
#define _CRC16_H_

#include <stdint.h>

uint16_t crc16(uint8_t *buf, uint16_t num);
uint16_t crc16f(uint8_t __flash *buf, uint16_t num);

#endif //_CRC16_H_
