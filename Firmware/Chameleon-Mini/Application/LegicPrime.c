/*
 * LegicPrime.c
 *
 *  Created on: 5.7.2024
 *      Author: Ladislav Marko
 *  Inspired by MifareClassic.c
 */
#if defined(CONFIG_LEGIC_PRIME_SUPPORT)

#include "LegicPrime.h"

char legic_log_str[64];
static uint8_t legic_iv_ack;
#include "../Codec/ISO14443-2F.h"
#include "../Memory.h"

#define LEGIC_PRIME_22_IV_ACK 0xD
#define LEGIC_PRIME_256_IV_ACK 0x1D
#define LEGIC_PRIME_1024_IV_ACK 0x3D

#define MEM_UID_ADDRESS         0x00
#define MEM_UID_CRC_ADDRESS     0x04
#define MEM_DCF_LOW_ADDRESS     0x05
#define MEM_DCF_HIGH_ADDRESS    0x06
#define MEM_BACKUP_ADDRESS      0x0D
#define MEM_BACKUP_CRC_ADDRESS  0x13

/* LEGIC prime card layout
 * UID [4 Bytes]
 * UID CRC [1 Byte]
 * Decremental filed  low byte (DCF) [1 byte]
 * Decremental filed  high byte (DCF) [1 byte]
 * 0x9F 0xFF 0x00 0x00 0x00 0x11 [6 bytes]
 * Backup [6 bytes]
 * Backup CRC [1 byte]
 * 0x00 0x00 [2 bytes]
 * additional segments
 * */

/*
 * Calculate transport CRC for LEGIC prime
 *
 * The CRC is calculated from the readers unmasked command concatenated with the card's data.
 * */
uint8_t calculateTransportCRC(long int data, int data_len){
    int state = 0x05;
    int polynomial = 0xC;
    int prev;
    for(int i = 0; i < data_len; i++){
        prev = state;
        state >>= 1;
        if ((prev ^ data) & 1){
            state ^= polynomial;
        }
        data >>= 1;
    }
    return state;
}

/* By Rich Schroeppel from HAKMEM */
uint8_t reflect8(uint8_t b) {
    return (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

/* Simplified Proxmark solution */
uint32_t calculateStorageCRC(uint8_t *buff, size_t size) {
    uint8_t mask = 0xFF;
    uint8_t state = 0x55 & mask;
    state = reflect8(state);
    for (size_t i = 0; i < size; ++i) {
        uint32_t data = buff[i];
        data = reflect8(data);
        state ^= data;
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (state & 0x80)
                state = (state << 1) ^ 0x63;
            else
                state = (state << 1);
        }
    }
    return state;
}


uint16_t LegicPrimeAppProcess(uint8_t *Buffer, uint16_t BitCount) {
    switch(BitCount){
        case 0:
            return ISO14443F_APP_NO_RESPONSE;
        case 7:
            // Probably start of setup phase
            //Type frame
            Buffer[0] = legic_iv_ack;
            return 6;
        case 6:
            // Probably end of setup phase
            return ISO14443F_APP_NO_RESPONSE;
        case 9:
            // Probably reading MIM256 card
            if (GetBitOnPositionInBuffer(Buffer, 0)){
                //Read command
                /* Retrieve the data from memory and calculate the CRC */
                long int data = 0;
                uint8_t address = ExtractByteFromPositionInBuffer(Buffer, 1);
                MemoryReadBlock(&data, MEM_UID_ADDRESS + address, 1);
                long int crc_input = (data << 9) | address << 1 | 0x1;


                /* Now put the data into the shared buffer and send them back to Codec */
                Buffer[0] = data;
                Buffer[1] = calculateTransportCRC(crc_input, 17);

                return 12;
            } else {
                //Write command
                //TODO: Implement
                return ISO14443F_APP_NO_RESPONSE;
            }
            // Fallthrough to case 11
        case 11:
            // Probably reading MIM1024 card
            if (GetBitOnPositionInBuffer(Buffer, 0)){
                /* Retrieve the data from memory and calculate the CRC */
                long int data = 0;
                uint16_t address = (ExtractByteFromPositionInBuffer(Buffer, 9) << 8 | ExtractByteFromPositionInBuffer(Buffer, 1)) & 0x3FF;

                MemoryReadBlock(&data, MEM_UID_ADDRESS + address, 1);
                long int crc_input = (data << 11) | address << 1 | 0x1;
//                sprintf(legic_log_str, "crc input is %lx ", crc_input);
//                LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));

                /* Now put the data into the shared buffer and send them back to Codec */
                Buffer[0] = data;
                Buffer[1] = calculateTransportCRC(crc_input, 19);
                return 12;
            } else {
                //Write command
                //TODO: Implement
                return ISO14443F_APP_NO_RESPONSE;
            }
        default:
            sprintf(legic_log_str, "Legic APP Processing unknown response");
            LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
            return ISO14443F_APP_NO_RESPONSE; //TODO: die horribly here?
    }
}

void LegicPrimeGetUid(ConfigurationUidType uid) {
    MemoryReadBlock(uid, MEM_UID_ADDRESS, LEGIC_PRIME_UID_SIZE);
}

void LegicPrimeSetUid(ConfigurationUidType uid) {
    MemoryWriteBlock(uid, MEM_UID_ADDRESS, LEGIC_PRIME_UID_SIZE);
    uint8_t storage_crc = calculateStorageCRC(uid, 4);
    MemoryWriteBlock(&storage_crc, MEM_UID_CRC_ADDRESS, 1);
}

void LegicPrimeAppInit22(void) {
    legic_iv_ack = LEGIC_PRIME_22_IV_ACK;
}

void LegicPrimeAppInit256(void) {
    legic_iv_ack = LEGIC_PRIME_256_IV_ACK;
}

void LegicPrimeAppInit1024(void) {
    legic_iv_ack = LEGIC_PRIME_1024_IV_ACK;
}

#endif
