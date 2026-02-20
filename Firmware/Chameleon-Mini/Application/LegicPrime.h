/*
 * LegicPrime.h
 *
 *  Created on: 5.7.2024
 *      Author: Ladislav Marko
 *  Inspired by MifareClassic.h
 */

#ifndef LEGIC_PRIME_H
#define LEGIC_PRIME_H
#include "Application.h"

#define LEGIC_PRIME_UID_SIZE       4
#define LEGIC_PRIME_22_MEM_SIZE       22
#define LEGIC_PRIME_256_MEM_SIZE       256
#define LEGIC_PRIME_1024_MEM_SIZE       1024

void LegicPrimeAppInit22(void);
void LegicPrimeAppInit256(void);
void LegicPrimeAppInit1024(void);

uint16_t LegicPrimeAppProcess(uint8_t *Buffer, uint16_t BitCount);
void LegicPrimeAppReset(void);

void LegicPrimeGetUid(ConfigurationUidType Uid);
void LegicPrimeSetUid(ConfigurationUidType Uid);
#endif //LEGIC_PRIME_H
