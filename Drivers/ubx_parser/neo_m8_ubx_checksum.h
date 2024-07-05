/*
 * neo_m8_ubx_checksum.h
 *
 *  Created on: Jul 17, 2018
 *      Author: alexis
 */

#ifndef NEO_M8_UBX_CHECKSUM_H_
#define NEO_M8_UBX_CHECKSUM_H_

#include <stdint.h>

/* Straightforward Solution */
uint16_t Fletcher16(uint8_t *data, int count);

/* Following UBX's documentation */
void UBX_Fletcher(uint8_t *data, int count, uint8_t *cka, uint8_t *ckb);

#endif /* NEO_M8_UBX_CHECKSUM_H_ */
