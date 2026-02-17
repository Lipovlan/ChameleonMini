/*
 * ISO14443-2F.h
 *
 *  Created on: 5.7.2024
 *      Author: Ladislav Marko
 *  Inspired by ISO14443-2A.h and ISO15693.h
 */

#ifndef ISO14443_F_H_
#define ISO14443_F_H_
#include "Codec.h"

#define ISO14443F_APP_NO_RESPONSE       0x0000

/* Codec Interface */
void ISO14443FCodecInit(void);
void ISO14443FCodecDeInit(void);
void ISO14443FCodecTask(void);

INLINE void SetBitOnPositionInBufferToValue(volatile uint8_t * buffer, uint16_t position, uint8_t value){
    uint16_t byte_offset = position / 8;
    uint8_t bit_offset = position % 8;

    if (value){
        buffer[byte_offset] |= (1 << bit_offset);        // Set bit to 1
    } else {
        buffer[byte_offset] &= ~(1 << bit_offset);       // Set bit to 0
    }
}

INLINE uint8_t GetBitOnPositionInBuffer(const uint8_t * buffer, uint16_t position){
    uint16_t byte_offset = position / 8;
    uint8_t bit_offset = position % 8;

    return (buffer[byte_offset] & (1 << bit_offset)) >> bit_offset;
}

INLINE uint8_t ExtractByteFromPositionInBuffer(const uint8_t * buffer, uint16_t position){
    //Be sure that this won't overflow
    //This function expects the buffer to be LSB first
    uint8_t byte = 0;
    for(uint8_t i = 0; i < 8; i++){
        byte <<= 1;
        uint8_t bit = GetBitOnPositionInBuffer(buffer, position + 7 - i);
        byte |= bit;
    }
    return byte;
}
#endif //ISO14443_F_H_
