/*
 * LegicPrime.c
 *
 *  Created on: 5.7.2024
 *      Author: Ladislav Marko
 *  Inspired by MifareClassic.c
 */
//#if defined(CONFIG_LEGIC_PRIME_SUPPORT)

#include "LegicPrime.h"

char legic_log_str[64];

#include "../Codec/ISO14443-2F.h"
#include "../Memory.h"

#define MEM_UID_ADDRESS         0x00
#define MEM_UID_CRC_ADDRESS     0x04
#define MEM_DCF_LOW_ADDRESS     0x05
#define MEM_DCF_HIGH_ADDRESS    0x06
#define MEM_BACKUP_ADDRESS      0x0D
#define MEM_BACKUP_CRC_ADDRESS  0x13

#define MEM_REPLAY_ADDRESS      0x1000

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
uint8_t response_index;

uint8_t calculateTransportCRC(uint8_t data){
    switch(data){
        case 0x81:
            return 0xA;
        case 0xAB:
            return 0x1;
        case 0xB8:
            return 0x8;
        case 0x4A:
            return 0xE;
        case 0xA7:
            return 0x0;
        default:
            return 0xF;
    }
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

uint16_t LegicPrimeAppProcess(uint8_t *Buffer, uint16_t BitCount) {

    uint8_t tmpbf[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    switch(BitCount){
        case 0:
            return ISO14443F_APP_NO_RESPONSE;
        case 7:
            // Probably start of setup phase
            //Type frame
            Buffer[0] = 0x1D; //0xd for MIM22, 0x1D for MIM256, 0x3D for MIM1024
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
                long int crc_input = (data << 9) | (Buffer[1] & 0x1) << 8 | Buffer[0];

                /* Now put the data into the shared buffer and send them back to Codec */
//                sprintf(legic_log_str, "CRC is %x made from %lx", calculateTransportCRC(crc_input, 17),  crc_input);
//                LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
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
            //TODO: Implement
            return ISO14443F_APP_NO_RESPONSE;
        default:
            sprintf(legic_log_str, "Legic APP Processing unknown response");
            LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
            return ISO14443F_APP_NO_RESPONSE; //TODO: die horribly here?
    }
}

void LegicPrimeGetUid(ConfigurationUidType Uid) {
    sprintf(legic_log_str, "LEGIC GET UID");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
    MemoryReadBlock(Uid, MEM_UID_ADDRESS, LEGIC_PRIME_UID_SIZE);
}

void LegicPrimeSetUid(ConfigurationUidType Uid) {
    sprintf(legic_log_str, "LEGIC SET UID");
    MemoryWriteBlock(Uid, MEM_UID_ADDRESS, LEGIC_PRIME_UID_SIZE);
    //TODO: Write also the LEGIC prime UID CRC to MEM_UID_CRC_ADDRESS
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
}

void LegicPrimeAppInit(void) {
    response_index = 0;
    sprintf(legic_log_str, "LEGIC APP INIT");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));

    uint8_t card_memory[] = {0x81, 0xAB, 0xB8, 0x4A, 0xA7};
    MemoryWriteBlock(card_memory, MEM_UID_ADDRESS, 5);

}

void LegicPrimeAppReset(void) {
    response_index = 0;
}
//#endif
