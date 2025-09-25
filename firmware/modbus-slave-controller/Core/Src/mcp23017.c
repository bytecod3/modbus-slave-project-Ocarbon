/*
 * mcp23017.c
 *
 *  Created on: Sep 25, 2025
 *      Author: eduh
 */


#include "mcp23017.h"

void MCP_initialize(MCP23017_instance inst, I2C_HandleTypeDef* i2c_handle, uint8_t address) {
	inst->address = address << 1;
	inst->i2c_handle = i2c_handle;
}

void MCP_pinmode(MCP23017_instance inst, uint8_t pin, uint8_t mode) {
	// resolve bank of the expander this pin is located
	uint8_t reg = (pin < 8) ? IODIRA : IODIRB;
	uint8_t reg_value[1];
	uint8_t p = pin % 8;

	// read the current register value
	HAL_I2C_Mem_Read(
			inst->i2c_handle,
			inst->address,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			reg_value,
			1,
			HAL_MAX_DELAY);

	// here we have the register value
	// set the pin mode (output is 0, input is 1)
	if(mode == INPUT) {
		reg_value[0] |= (1 << p);

	} else if(mode == OUTPUT) {
		reg_value[0] &= ~(1 << p);
	}

	// update the registers
	HAL_I2C_Mem_Write(
			inst->i2c_handle,
			inst->address,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			reg_value,
			1,
			HAL_MAX_DELAY);

}

void MCP_write_pin(MCP23017_instance inst, uint8_t pin, uint8_t level) {
	// resolve the output LATCH port
	uint8_t reg = (pin < 8) ? OLATA : OLATB;
	uint8_t reg_value[1];
	uint8_t p = (pin % 8);

	// read from the port register
	HAL_I2C_Mem_Read(
				inst->i2c_handle,
				inst->address,
				reg,
				I2C_MEMADD_SIZE_8BIT,
				reg_value,
				1,
				HAL_MAX_DELAY);

	// modify the pin
	if(level == HIGH) {
		reg_value[0] |= (1 << p);
	} else if(level == LOW) {
		reg_value[0] &= ~(1 << p);
	} else {
		// todo: handle invalid level
	}

	// write back
	// update the registers
	HAL_I2C_Mem_Write(					// todo: put this in its own function to maintain DRY
			inst->i2c_handle,
			inst->address,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			reg_value,
			1,
			HAL_MAX_DELAY);

}

/**
 * read a single pin
 */
uint8_t MCP_read_pin(MCP23017_instance inst, uint8_t pin) {
	// resolve the GPIO to read from
	uint8_t reg = (pin < 8) ? MCP_GPIOA : MCP_GPIOB;
	uint8_t reg_value[1];

	// get the pin number
	uint8_t p = (pin % 8);

	// read the current register value
	HAL_I2C_Mem_Read(
			inst->i2c_handle,
			inst->address,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			reg_value,
			1,
			HAL_MAX_DELAY);

	// extract the value of the pin
	uint8_t val = (reg_value[0] >> p) & 0x01;
	return val;
}

/**
 * Read the whole GPIO port
 * @param port The port to read from -> use 0 for PORTA, and 1 for port B
 */
uint8_t MCP_read_port(MCP23017_instance inst, uint8_t port_num) {
	// resolve the GPIO to read from
	uint8_t reg;
	if(port_num == 1) {
		reg = MCP_GPIOA;
	} else if(port_num == 0) {
		reg = MCP_GPIOB;
	} else {
		// todo: return a debug value
	}

	uint8_t reg_value[1];

	// read the current register value
	HAL_I2C_Mem_Read(
			inst->i2c_handle,
			inst->address,
			reg,
			I2C_MEMADD_SIZE_8BIT,
			reg_value,
			1,
			HAL_MAX_DELAY);

	return reg_value[0];
}



