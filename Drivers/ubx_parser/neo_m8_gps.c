/*
 * neo_m8_gps.c
 *
 *  Created on: Jul 17, 2018
 *      Author: alexis
 */

#include "neo_m8_gps.h"

/* Variables */
/* Ring buffer to store incoming bytes ; raw data storage */
static ring_buffer_t gps_rx_ring_buffer;

static char UBXPayload[128];
static UBXHeader header;
static char CRC_A;
static char CRC_B;

static UBXMsg ValidUBXMsg;

// todo: remove
static char gps_raw_buffer[200]; /* Buffer used to extract msg from ring buffer */
static char dbg_buf[128];

/* Initialisation function */
void neoInit(void)
{
	const uint8_t disable_spi_i2c[] = {0xb5, 0x62, 0x06, 0x8a, 0x0e, 0x00, 0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0x00, 0x06, 0x00, 0x64, 0x10, 0x00, 0x7d, 0x03};
	const uint8_t disable_nmea_enable_ubx[] = {0xb5, 0x62, 0x06, 0x8a, 0x0e, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x74, 0x10, 0x01, 0x02, 0x00, 0x74, 0x10, 0x00, 0xab, 0x29};
	// uint8_t set_timepulse[] = {0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x50, 0xC3, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0xA6};
	uint8_t enable_nav_pvt[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01, 0x53, 0x48}; // rate=1
	uint8_t enable_nav_odo[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x7f, 0x00, 0x91, 0x20, 0x01, 0xcb, 0xa0}; // rate=1
	uint8_t enable_nav_dop[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x39, 0x00, 0x91, 0x20, 0x01, 0x85, 0x42}; // rate=1
	uint8_t enable_sat_msg[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x16, 0x00, 0x91, 0x20, 0x01, 0x62, 0x93}; // rate=1
	uint8_t odr5hz[] = {0xb5, 0x62, 0x06, 0x8a, 0x0a, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0xc8, 0x00, 0xb5, 0x81};   // 200 ms measurement time => 5 Hz (Note: faster ODR may result in u-center GUI flashing)

// for (size_t i = 0; i < sizeof(enable_nav_pvt); i++)
// {
// 	enable_nav_pvt[i] = enable_nav_pvt[i] | enable_nav_dop[i] | enable_nav_odo[i];
// }

// No UBX-NAV-ATT message on NEO-M9N
#define TIMEOUT 100000
#define BRUTE_FORCE(expr)           \
	for (size_t i = 0; i < 10; i++) \
	{                               \
		expr;                       \
	}
	HAL_StatusTypeDef ok = 0;
	// FIXME: HACK: Some issue with the commands not being received, resending them 10 times works but we should probably check the configuration before proceeding.
	// BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, disable_spi_i2c, sizeof(disable_spi_i2c), TIMEOUT));
	HAL_Delay(1);
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, disable_nmea_enable_ubx, sizeof(disable_nmea_enable_ubx), TIMEOUT));
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, enable_nav_pvt, sizeof(enable_nav_pvt), TIMEOUT));
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, odr5hz, sizeof(odr5hz), TIMEOUT));
	// ok |= HAL_UART_Transmit(&huart3, set_timepulse, sizeof(set_timepulse), TIMEOUT);
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, enable_nav_odo, sizeof(enable_nav_odo), TIMEOUT));
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, enable_nav_dop, sizeof(enable_nav_dop), TIMEOUT));
	BRUTE_FORCE(ok |= HAL_UART_Transmit(&huart3, enable_sat_msg, sizeof(enable_sat_msg), TIMEOUT));

	//
	/* Init the reception buffer */
	neoInitRxBuf();

	/* Enable RX Not Empty interrupt */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

void neoInitRxBuf(void)
{
	ring_buffer_init(&gps_rx_ring_buffer);
}

uint32_t neoNumElemBuf(void)
{
	return (uint32_t)ring_buffer_num_items(&gps_rx_ring_buffer);
}

/* Rx callback to handle the reception of one byte */
void neoRxCallback(char c)
{
	ring_buffer_queue(&gps_rx_ring_buffer, c);
}

UBX_OP_RESULT neoRetrieveMsg(void)
{
	static UBX_NEO_M8_DECODE_STATE smState = UBX_LOOKING_FOR_SYNC;
	switch (smState)
	{
	case UBX_LOOKING_FOR_SYNC:
	{
		if (neoFindSyncBytes())
		{
			smState = UBX_BUILDING_MSG_INFOS;
			/* Flow through */
		}
		else
		{
			return UBX_NO_DATA;
			break;
		}
	}

	case UBX_BUILDING_MSG_INFOS:
	{
		if (neoBuildHeader(&header))
		{
			/* Retrieved a header (informations of the UBX Packet: ID, Class, Len) */
			/* Check the length, in case a error a very high value can be reached */
			if (header.length > 172) // todo: define the max length of a ubx payload
			{
				/* That's too high, switch to previous state again */
				smState = UBX_LOOKING_FOR_SYNC;
				break;
			}
			else
			{
				/* Valide length, allow flow through the next state */
				smState = UBX_CHECKING_CRC;
			}
		}
		else
		{
			break;
		}
	}

	case UBX_CHECKING_CRC:
	{
		int i;
		char rxd_crca, rxd_crcb;
		/* Check if enough data is present inside the buffer for this operation */
		/* Header (4 bytes) + Payload (length bytes) + CRC (2 bytes) = length + 6 */
		/* Last two bytes are the CRC, we won't copy it into the raw buffer, but in separate values */
		if (ring_buffer_num_items(&gps_rx_ring_buffer) >= (header.length + 6))
		{
			if (!neoCopyPacketFromRing(gps_raw_buffer, header.length + 4))
			{
				/* Error to handle */
				break;
			}
			/* Arrive here only if copy was successfull and CRC is available in ring buffer */
			/* Verify checksum is correct */

			/* Retrieve Rxd CRC*/
			ring_buffer_peek(&gps_rx_ring_buffer, &rxd_crca, (header.length + 4));
			ring_buffer_peek(&gps_rx_ring_buffer, &rxd_crcb, (header.length + 5));

			/* Calculate CRC on Header (class + id + length) + payload */
			UBX_Fletcher(gps_raw_buffer, header.length + 4, &CRC_A, &CRC_B);

			/* In any case, this is the end and it will be time to look for new SYNC bytes */
			smState = UBX_LOOKING_FOR_SYNC;

			/* Check CRC */
			if ((CRC_A == rxd_crca) && (CRC_B == rxd_crcb))
			{
				/* We retrieved a correct message, remove it from ring buffer */
				neoDequeueFromRing(header.length + 6);
				memset(ValidUBXMsg.payload, 0, 172);
				memcpy(ValidUBXMsg.payload, &gps_raw_buffer[4], header.length);
				ValidUBXMsg.header = header;

				return UBX_NEW_DATA;
			}
			else
			{
				/* Else don't remove it and try to find a SYNC
				 * A packet could be present in the byte we dealt with, that's why we only peek'd */
				return UBX_INCORRECT_PACKET_RXD;
			}
		}
		else
		{
			/* Not enough item in the ring buffer, let him get feed */
			return UBX_WAITING_MORE_DATA;
			break;
		}

		/* Should never get there */
		break;
	}
	}
}

/* Browse into the RX Ring buffer to find SYNC Char 1 followed by SYNC Char 2, indicating the begin of a UBX msg */
/* Returns true if we retrieved this combination, false otherwise */
bool neoFindSyncBytes()
{
	char c1, c2;
	while (!ring_buffer_is_empty(&gps_rx_ring_buffer) && (ring_buffer_num_items(&gps_rx_ring_buffer) >= 2))
	{
		ring_buffer_dequeue(&gps_rx_ring_buffer, &c1);
		if (c1 == UBX_SYNC_CHAR1)
		{
			ring_buffer_peek(&gps_rx_ring_buffer, &c2, 0);
			if (c2 == UBX_SYNC_CHAR2)
			{
				ring_buffer_dequeue(&gps_rx_ring_buffer, &c2); /* Dequeue the byte we just peek'd before */
				return true;								   /* We just found SYNC1 followed by SYNC2 */
			}
		}
	}
	return false;
}

bool neoRetrieveHeaderBytes(char *buf)
{
	int i;

	for (i = 0; i < 4; i++)
	{
		if (!ring_buffer_peek(&gps_rx_ring_buffer, buf++, i))
		{
			/* The ring_buffer_peek will return false if the index don't exist
			 * it means that there are less than 4 bytes! */
			return false;
		}
	}

	/* Correctly peeked 4 bytes from the ring buffer */
	return true;
}

bool neoBuildHeader(UBXHeader *header)
{
	// memset(header->rawBuf, 0, 4);

	if (neoRetrieveHeaderBytes(gps_raw_buffer))
	{
		/* Successfully retrieved 4 bytes to build a header */
		header->class = gps_raw_buffer[0];
		header->id = gps_raw_buffer[1];
		header->length = gps_raw_buffer[3] << 8 | gps_raw_buffer[2]; /* Little Endian */

		return true;
	}
	else
	{
		return false;
	}
}

bool neoDequeueFromRing(uint32_t n)
{
	char dummy;
	if (ring_buffer_num_items(&gps_rx_ring_buffer) >= n)
	{
		// dequeue
		for (int i = 0; i > n; i++)
		{
			ring_buffer_dequeue(&gps_rx_ring_buffer, &dummy);
		}
	}
}

/* This function copies bytes from a ring buffer to the raw buffer.
 * Second argument is a pointer to the raw buffer,
 * Third argument is the number of bytes to copy
 * Returns true if the copy occured correctly,
 * Returns false if something went wrong (not enough elements in the ring buffer)
 * Use this function after the sync byte were detected and dequeue'd like this:
 */
bool neoCopyPacketFromRing(char *rbuffer, uint32_t n)
{
	char dummy;
	int i;
	if (ring_buffer_num_items(&gps_rx_ring_buffer) >= n)
	{
		for (i = 0; i < (header.length + 4); i++)
		{
			if (!ring_buffer_peek(&gps_rx_ring_buffer, &gps_raw_buffer[i], i))
			{
				/* Oops, no item at the index we indicated */
				return false;
			}
		}
	}
	else
	{
		return false;
	}
	/* OK */
	return true;
}

uint8_t neoGetMsgClass()
{
	return ValidUBXMsg.header.class;
}

uint8_t neoGetMsgId()
{
	return ValidUBXMsg.header.id;
}

uint16_t neoGetPayloadSize()
{
	return ValidUBXMsg.header.length;
}

void UBXUpdate_NAV_PVT(ubx_nav_pvt_msg_t *dest)
{
	UBX_Parse_Raw_To_NAV_PVT(ValidUBXMsg.payload, ValidUBXMsg.header.length, dest);
}

void UBXUpdate_NAV_DOP(ubx_nav_dop_msg_t *dest)
{
	UBX_Parse_Raw_To_NAV_DOP(ValidUBXMsg.payload, ValidUBXMsg.header.length, dest);
}
