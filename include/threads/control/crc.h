#ifndef RM2024_THREADS_SERIAL_CRC_H_
#define RM2024_THREADS_SERIAL_CRC_H_

// Robomaster 官方提供的代码

#include <cstdint>
#include <cstddef>
#include <cstdio>

extern "C" {

uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);

uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

int print_bytes(void *data, size_t len);

} 


#endif