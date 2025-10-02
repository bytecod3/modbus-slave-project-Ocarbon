/*
 * max485.c
 *
 *  Created on: Sep 26, 2025
 *      Author: eduh
 */

#include <modbus_rtu.h>

/**
 * @brief this function initializes the MAX485 instance
 */
void MAX485_init(MAX485_instance inst, uint8_t pin) {
	inst->DE_RE_pin = pin;
}

/**
 * @brief This functions enables the transmit mode on MAX485 IC
 */
void MAX485_enable_transmit(MAX485_instance inst) {
	// write 1 on the DE pin
	HAL_GPIO_WritePin(inst->DE_RE_PORT, inst->DE_RE_pin, GPIO_PIN_SET);

}

void MAX485_enable_receive(MAX485_instance inst) {
	// write 1 on the RE pin
	HAL_GPIO_WritePin(inst->DE_RE_PORT, inst->DE_RE_pin, GPIO_PIN_RESET);
}

/**
 * @brief This function receives the UART data as a buffer and decodes it
 */
void MAX485_receive(MAX485_instance inst, uint8_t* data, uint16_t len, uint32_t timeout) {
	// enable receive
	MAX485_enable_receive(inst);

	// check the expected length
	// confirm data integrity
	// extract the desired
}

uint16_t MAX485_calculate_CRC(const uint8_t* buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for(uint16_t pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buf[pos];

		for(int i = 0; i < 8; i++) {
			if(crc & 0x0001) crc = (crc>>1) ^ 0xA001;
			else crc >>= 1;
		}
	}

	return crc;
}

/**
 * @brief This function handles rea coils function
 */
//void MAX_485_read_coils_handler() {
//	HAL_UART_Transmit(&huart, pData, Size, Timeout)
//}
