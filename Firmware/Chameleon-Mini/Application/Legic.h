//
// Created by l on 6.8.24.
//

#ifndef LEGIC_H
#define LEGIC_H
#include "Application.h"
#include "ISO14443-3A.h"

#define LEGIC_UID_SIZE       ISO14443A_UID_SIZE_SINGLE
#define LEGIC_MEM_SIZE       1024 // There are two LEGIC prime variants -- 256 and 1024 bytes, so we use the bigger one

void LegicAppInit(void);
void LegicAppReset(void);
void LegicAppTask(void);

uint16_t LegicAppProcess(uint8_t *Buffer, uint16_t BitCount);

void LegicGetUid(ConfigurationUidType Uid);
void LegicSetUid(ConfigurationUidType Uid);
#endif //LEGIC_H
