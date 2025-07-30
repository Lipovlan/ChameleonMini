//
// Created by l on 6.8.24.
//
#if defined(CONFIG_LEGIC_SUPPORT)

#include "Legic.h"

char legic_log_str[64];

// ============================== From MifareClassic.c

#include "ISO14443-3A.h"
#include "../Codec/ISO14443-F.h"
#include "../Memory.h"
#include "Crypto1.h"
#include "../Random.h"

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



uint16_t LegicAppProcess(uint8_t *Buffer, uint16_t BitCount) {
    uint8_t tmpbf[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    //                |                   DATA               |         CRC       |
    switch(BitCount){
        case 7:
            // Probably start of setup phase
            tmpbf[0] = 0x1; tmpbf[3] = 0x1; tmpbf[4] = 0x1;
            memcpy(CodecBuffer, tmpbf, 6);
            return 6;
        case 6:
            // Probably end of setup phase
            return ISO14443F_APP_NO_RESPONSE;
        case 9:
            // Probably reading MIM256 card
            // Fallthrough to case 11
        case 11:
            // Probably reading MIM1024 card
            tmpbf[0] = 0x1; tmpbf[1] = 0x1; tmpbf[2] = 0x1; tmpbf[5] = 0x1; tmpbf[8] = 0x1; tmpbf[9] = 0x1; tmpbf[11] = 0x1;
            memcpy(CodecBuffer, tmpbf, 12);
            return 12;
        default:
            return ISO14443F_APP_NO_RESPONSE; //TODO: die horribly here?
    }
}

void LegicGetUid(ConfigurationUidType Uid) {
    sprintf(legic_log_str, "LEGIC GET UID");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
    Uid[0] = 0xAA;
    Uid[1] = 0xBB;
    Uid[2] = 0xCC;
    Uid[3] = 0xDD;
}

void LegicSetUid(ConfigurationUidType Uid) {
    sprintf(legic_log_str, "LEGIC SET UID");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));
}
void LegicAppInit(void) {
    sprintf(legic_log_str, "LEGIC APP INIT");
    LogEntry(LOG_INFO_GENERIC, legic_log_str, strlen(legic_log_str));

    State = STATE_IDLE;
}

void LegicAppReset(void) {
    State = STATE_IDLE;
}

// ============================== end MifareClassic.c
#endif
