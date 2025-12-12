//
// Created by l on 6.8.24.
//
#if defined(CONFIG_LEGIC_PRIME_SUPPORT)

#include "LegicPrime.h"

char legic_log_str[64];

// ============================== From MifareClassic.c

#include "ISO14443-3A.h"
#include "../Codec/ISO14443-2F.h"
#include "../Memory.h"
#include "Crypto1.h"
#include "../Random.h"

#define MEM_UID_ADDRESS         0x00
#define MEM_UID_CRC_ADDRESS     0x04
#define MEM_DCF_LOW_ADDRESS     0x05
#define MEM_DCF_HIGH_ADDRESS    0x06
#define MEM_BACKUP_ADDRESS      0x0D
#define MEM_BACKUP_CRC_ADDRESS  0x13

uint8_t response_index;

static enum {
    STATE_HALT,
    STATE_IDLE,
    STATE_CHINESE_IDLE,
    STATE_CHINESE_WRITE,
    STATE_READY1,
    STATE_READY2,
    STATE_ACTIVE,
    STATE_AUTHING,
    STATE_AUTHED_IDLE,
    STATE_WRITE,
    STATE_INCREMENT,
    STATE_DECREMENT,
    STATE_RESTORE
} State;


uint16_t LegicPrimeAppProcess(uint8_t *Buffer, uint16_t BitCount) {

uint16_t LegicAppProcess(uint8_t *Buffer, uint16_t BitCount) {
    uint8_t tmpbf[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    //                |                   DATA               |         CRC       |
   // RIGHT NOW 0x81 0xAB 0xB8 0x4A
    switch(BitCount){
        case 7:
            // Probably start of setup phase
            tmpbf[0] = 0x19;
            memcpy(CodecBuffer, tmpbf, 1);
            return 6;
        case 6:
            // Probably end of setup phase
            return ISO14443F_APP_NO_RESPONSE;
        case 9:
            // Probably reading MIM256 card
            // Fallthrough to case 11
        case 11:
            // Probably reading MIM1024 card
            switch(response_index){
                case 0:
                    tmpbf[0] = 0x27; tmpbf[1] = 0xB; response_index++; break;
                case 1:
                    tmpbf[0] = 0x9B; tmpbf[1] = 0x1; response_index++; break;
                case 2:
                    tmpbf[0] = 0xA1; tmpbf[1] = 0x1; response_index++; break;
                case 3:
                    tmpbf[0] = 0x6D; tmpbf[1] = 0xA; response_index++; break;
                case 4:
                    tmpbf[0] = 0x52; tmpbf[1] = 0x3; response_index++; break;
            }

            memcpy(CodecBuffer, tmpbf, 2);
            return 12;
        default:
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
    State = STATE_IDLE;
}

void LegicPrimeAppReset(void) {
    response_index = 0;
    State = STATE_IDLE;
}

// ============================== end MifareClassic.c
#endif
