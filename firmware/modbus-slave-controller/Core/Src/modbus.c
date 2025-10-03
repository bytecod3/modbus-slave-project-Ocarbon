/*
 * modbus.c
 *
 *  Created on: Oct 3, 2025
 *      Author: eduh
 */

#include "modbus.h"
#include "modbus_rtu.h"

/* max coils we can hold */
#define COIL_COUNT 100

/* abyte array of coils */
uint8_t COIL[COIL_COUNT] = {0};

/* example coils byte array for testing - each bit holds a coil - grouped into bytes of coils */
uint8_t coils[(COIL_COUNT + 7) / 8] = {0x4D, 0x0D};

/**
 * @brief This function handles MODBUS function codes, whether they are from RTU or TCP
 */
uint16_t MODBUS_handle_function(uint8_t slave_id, uint8_t function_code, uint8_t* modbus_data, uint8_t* response, uint8_t src) {

	/*todo: check for null pointer */
	if(response != NULL) {

		/* to hold the length of the computed response. This is dynamic based on the function code */
		uint16_t response_length = 0;

		switch (function_code) {
			case 0x01:

				/* for debug */
				//HAL_UART_Transmit(&huart1, (uint8_t*) "READ COILS\r\n", strlen("READ COILS \r\n"), HAL_MAX_DELAY);

				/* coil to start from */
				uint16_t start = (modbus_data[2] << 8) | (modbus_data[3]);

				/* how many coils */
				uint16_t qty = (modbus_data[4] << 8) | (modbus_data[5]);

				/* todo: check valid coils - build exception */
				response[0] = slave_id;
				response[1] = 0x01;

				/* 7 ensures a proper roundup to get number of bytes needed to hold the requested number of bits */
				response[2] = (qty + 7)/8;

				uint16_t index = 3;
				uint8_t coil_byte = 0;
				uint8_t bit_pos = 0;

				for(uint16_t i = 0; i < qty; i++) {
					/* get the actual coil value - bit extraction*/
					/* this is using a simulated coils buffer that have prefixed coil status - ideally it should be dynamic from the relays */
					uint8_t coil_val = (coils[(start + i) / 8] >>  ( (start + i) %8) ) & 0x01;

					/* if coil is set, set coil byte value */
					if(coil_val) {
						coil_byte |= (1 << bit_pos);
					}

					/* next bit position */
					bit_pos++;

					/* we have reached end of byte. reset to next byte */
					if(bit_pos == 8 || i == qty - 1) {
						response[index++] = coil_byte;
						coil_byte = 0;
						bit_pos = 0;
					}

				}

				/* append CRC for RTU messages */
				if(src == MODBUS_RTU) {
					uint16_t crc = MAX485_calculate_CRC(response, index);

					/* get CRC LOW */
					response[index++] = crc & 0xFF;

					/* get CRC HIGH */
					response[index++] = (crc >> 8) & 0xFF;

					response_length = index;

				} else {
					/* modbus TCP */
				}


				break;


			case 0x05:
				break;

		}

		return response_length;

	}



}
