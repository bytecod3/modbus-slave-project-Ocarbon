/*
 * max485.c
 *
 *  Created on: Sep 26, 2025
 *      Author: eduh
 */

#include <modbus_rtu.h>
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_gpio.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

/**
 * create holding registers
 */
uint16_t holding_registers[NUM_HOLDING_REGISTERS];

/**
 * @brief This function create and initializes holding registers
 * Register Map todo: create register map
 *
 *
 *
 */
void modbus_rtu_init_holding_regs(uint16_t holding_regs[], int size) {

	// firmware version
	holding_regs[0] = FIRMWARE_VERSION;
	holding_regs[1] = SLAVE_ID;

	for(int i = 3; i < size; i++) {
		holding_regs[i] = 0;
	}
}

/**
 * @brief this function initializes the MAX485 instance
 */
void MAX485_init(MAX485_instance inst) {
	inst->uart_instance = &huart2;
	inst->de_port = DE_RE_PIN_GPIO_Port;
	inst->de_pin = DE_RE_PIN_Pin;

	// set default mode to receive
	HAL_GPIO_WritePin(inst->de_port, inst->de_pin, GPIO_PIN_RESET);
}

/**
 * @brief This functions enables the transmit mode on MAX485 IC
 */
void MAX485_enable_transmit(MAX485_instance inst) {
	// write 1 on the DE pin
	HAL_GPIO_WritePin(inst->de_port, inst->de_pin, GPIO_PIN_SET);

}

void MAX485_enable_receive(MAX485_instance inst) {
	// write 1 on the RE pin
	HAL_GPIO_WritePin(inst->de_port, inst->de_pin, GPIO_PIN_RESET);
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

