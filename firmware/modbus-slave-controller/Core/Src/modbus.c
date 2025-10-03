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
			case READ_COIL:

				/* for debug */
				//HAL_UART_Transmit(&huart1, (uint8_t*) "READ COILS\r\n", strlen("READ COILS \r\n"), HAL_MAX_DELAY);

				/* coil to start from */
				uint16_t start = (modbus_data[2] << 8) | (modbus_data[3]);

				/* how many coils to read */
				uint16_t qty = (modbus_data[4] << 8) | (modbus_data[5]);

				/* todo: check valid coils - build exception */
				response[0] = slave_id;
				response[1] = 0x01;

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

				/* we have the number of bytes we produced */
				response[2] = index - 3;

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

			case FORCE_SINGLE_COIL:

				/* get the start address */
				uint16_t start_address = (modbus_data[2] << 8) | (modbus_data[3]);
				uint16_t coil_stte = (modbus_data[4] << 8) | (modbus_data[5]);

				uint8_t byte_indx = start_address / 8;
				uint8_t bit_indx = start_address % 8;

				if(coil_stte == 0xFF00) {
					/* simulate coil setting */
					coils[byte_indx] |= (1 << bit_indx);
				} else if(coil_stte == 0x00) {
					coils[byte_indx] &= ~(1 << bit_indx);
				} else {
					/* deal with dircty data */
				}

				response[0] = slave_id;
				response[1] = function_code;
				response[2] = modbus_data[2] << 8;
				response[3] = modbus_data[3];
				response[4] = coil_stte >> 8; /* value high */
				response[5] = coil_stte & 0xFF; /* value low */

				/* compute crc */
				uint16_t crc16 = MAX485_calculate_CRC(response, 6);
				response[6] = crc16 & 0xFF ;
				response[7] = crc16 >> 8;

				response_length = 8;

				return response_length;

				break;

			case FORCE_MULTIPLE_COILS:

				uint16_t start_addr = (modbus_data[2] << 8) | modbus_data[3];

				/* number of coils to force */
				uint16_t quty = (modbus_data[4] << 8) | (modbus_data[5]);

				/* number of byte counts that follow */
				uint8_t num_bytes = modbus_data[6];

				/* get the actual coil data */
				uint8_t* coil_data = modbus_data[7];

				uint8_t coil_state = 0;

				/* coil data */
				for(uint16_t i = 0; i < quty; i++) {
					uint8_t byte_index = i / 8;
					uint8_t bit_index = i % 8;

					/* this is whatever we have received from master */
					coil_state = (coil_data[byte_index] >> bit_index) & 0x01;

					uint16_t target_index = start_addr + i;
					uint16_t target_byte = target_index / 8;
					uint8_t target_bit = target_index % 8;

					/* simulate coil setting and resetting */
					if(coil_state) {
						coils[target_byte] |= (1 << target_bit); /* ON */
					} else {
						coils[target_byte] &= ~(1 << target_bit); /* OFF */
					}

				}

				/* compute the response */
				response[0] = slave_id ;
				response[1] = function_code;
				response[2] = (start_addr >> 8);
				response[3] = (start_addr & 0xFF);
				response[4] = (quty >> 8);
				response[5] = (quty & 0xFF);

				/* calculate CRC */
				uint16_t crc = MAX485_calculate_CRC(response, 6);
				response[6] = crc & 0xFF ;
				response[7] = crc >> 8;

				response_length = 8;

				break;


			case REPORT_SLAVE_ID:
				break;

		}

		return response_length;

	}



}
