/**
 * This code computes CRC of a given shar array 
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

uint16_t crc16(const char* buffer, uint16_t len) {
	// find the length of the buffer 
	uint16_t crc = 0xFFFF; // 16 bit
	for(uint16_t pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buffer[pos];

		for(int i = 0; i < 8; i++) {
			if(crc & 0x0001) {
				crc = (crc >> 1) ^ 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;

}

int main() {
	// frame withour CRC 
	uint8_t frame[] = {
		0x01,
		0x01,
		0x00,
		0x00,
		0x00,
		0x08,
		0x00, //placeholder for CRC LO
		0x00, //placeholder for CRC HI
	};

	uint16_t crc = crc16(frame, sizeof(frame)-2);

	printf("CRC: %0x\r\n", crc);

	// append to end if frame 
	size_t frame_len = sizeof(frame) / sizeof(frame[0]);
	frame[6] = crc & 0xFF;
	frame[7] = (crc >> 8) & 0xFF;

	for(int i = 0; i < frame_len; i++) {
		printf(" %0x \n", frame[i]);
	}

	return 0;

}
