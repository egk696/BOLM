/*
 * XBee.h
 *
 *  Created on: Aug 9, 2017
 *      Author: chris
 */

#ifndef XBEE_H_
#define XBEE_H_
/*
#include <stdint.h>
#include <stdlib.h>
#include <string.h>



typedef uint64_t XBeeAddress64;

typedef struct ZBTxHeader {
	uint8_t startByte;
	uint16_t length;
	uint8_t frameType;
	uint8_t frameID;
	XBeeAddress64 address;
	uint16_t destinationNetworkAddr;
	uint8_t broadcastRadius;
	uint8_t otpions;
} ZBTxHeader;

typedef struct ZBTxPayload {
	int payloadSize;
	uint8_t* payload;
} ZBTxPayload;

//API Mode of XBee (atap1 or atap2)
#define ATAP 1

#define START_BYTE 0x7e
#define TX_FRAME_LENGTH 16


uint64_t htonll(uint64_t value);
uint8_t ZBTxRequest(XBeeAddress64, uint8_t *payload, uint8_t payloadLength);

#endif /* XBEE_H_ */
*/
