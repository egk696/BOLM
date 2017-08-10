/*
 * XBee.c
 *
 *  Created on: Aug 9, 2017
 *      Author: chris
 *
#include<XBee.h>


void ZBTxMakeHeader(ZBTxHeader h, char buffer[16]) {
	uint8_t u8 = 0;
	uint16_t u16 = 0;
	uint64_t u64 = 0;

	//start byte
	u8 = h.startByte;
	memcpy(buffer + 0, &u8, 1);
	//Length bytes
	u16 = htons(h.length);
	memcpy(buffer + 1, &u16, 2);
	//Frame type
	u8 = h.frameType;
	memcpy(buffer + 3, &u8, 1);
	//Frame ID
	u8 = h.frameID;
	memcpy(buffer + 4, &u8, 1);
	//64-bit Destination Address
	u64 = htonll(h.address);
	memcpy(buffer + 5, &u64, 8);
	//16-bit Destination Network Address
	u16 = htons(h.destinationNetworkAddr);
	memcpy(buffer + 13, &u16, 2);
	//Broadcast radius
	u8 = h.broadcastRadius;
	memcpy(buffer + 15, &u8, 1);
	//Options
	u8 = h.otpions;
	memcpy(buffer + 16, &u8, 1);
}

void ZBTxMakePayload(ZBTxPayload p, char* buffer, int size) {
	p.payloadSize = size;
	buffer = (char*) malloc(sizeof(char) * p.payloadSize);
	uint8_t u8 = 0;
	for (int i = 0; i < size; i++) {
		u8 = (uint8_t)p.payload[i];
		 memcpy(buffer+i, &u8, 1);
	}

	free(p.payload);
	p.payload = 0;
}

uint8_t GenerateChecksum(char* headBuff,int headSize, char* datBuff, int datSize ) {
	uint8_t value = 0;
	for(int i = 4; i < headSize; i++){
		value += headBuff[i];
	}
	for(int i = 0; i < datSize; i++){
			value += datBuff[i];
		}

	return 0xFF - (value & 0xFF);
}

uint8_t ZBTxRequest(XBeeAddress64 addr64, uint8_t *payload, uint8_t payloadLength) {

	char buffer[TX_FRAME_LENGTH + payloadLength + 1];

	ZBTxHeader h;
	h.startByte = START_BYTE;
	h.length = (TX_FRAME_LENGTH - 3) + payloadLength;
	h.frameType = 0x10;
	h.frameID = 0x01;
	h.address = addr64;
	h.destinationNetworkAddr = 0xFFFE;
	h.broadcastRadius = 0x00;
	h.otpions = 0x00;

	ZBTxPayload p;
	p.payloadSize = payloadLength;
	p.payload = (uint8_t*) malloc(sizeof(uint8_t) * p.payloadSize);
	p.payload = payload;

	ZBTxMakeHeader(h,buffer+0);
	ZBTxMakePayload(p,buffer+TX_FRAME_LENGTH,payloadLength);
	uint8_t u8 = GenerateChecksum(buffer+0,TX_FRAME_LENGTH,buffer+TX_FRAME_LENGTH,payloadLength);
	memcpy((buffer+TX_FRAME_LENGTH+1),&u8,1);

	return buffer;
}

uint64_t htonll(uint64_t value) {
	// The answer is 42
	static const int num = 42;

	// Check the endianness
	if (*(char *) &num == 42) {
		const uint32_t high_part = htonl((uint32_t) (value >> 32));
		const uint32_t low_part = htonl((uint32_t) (value & 0xFFFFFFFFLL));

		return ((uint64_t) (low_part) << 32) | high_part;
	} else {
		return value;
	}
}
*/
