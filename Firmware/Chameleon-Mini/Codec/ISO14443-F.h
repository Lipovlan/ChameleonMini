/*
 * ISO14443-F.h
 *
 *  Created on: 5.7.2024
 *      Author: l
 *      Based on: ISO14443-2A.h
 */

#ifndef ISO14443_F_H_
#define ISO14443_F_H_
#include "Codec.h"

/* Codec Interface */
void ISO14443FCodecInit(void);
void ISO14443FCodecDeInit(void);
void ISO14443FCodecTask(void);

#endif //ISO14443_F_H_
