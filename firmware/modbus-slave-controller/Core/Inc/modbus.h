/*
 *
 * @file modbus.h
 * @brief contains shared functions between MODBUS RTU and MODBUS TCP
 * @date Created on: Oct 3, 2025
 * @author: Edwin
 *
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "stm32f4xx_hal.h"

/* MODBUS function codes */
enum function_codes {
	READ_COIL 					= 0x01,
	READ_HOLDING_REGISTER 		= 0x03,
	READ_INPUT_REGISTER 		= 0x04,
	FORCE_SINGLE_COIL 			= 0x05,
	PRESET_SINGLE_REGISTER 		= 0x06,
	FORCE_MULTIPLE_COILS 		= 0x15,
	PRESET_MULTIPLE_REGISTERS	= 0x16,
	REPORT_SLAVE_ID 			= 0x17
};

#define MODBUS_RTU (0x00)
#define MODBUS_TCP (0x02)

uint16_t MODBUS_handle_function(uint8_t slave_id, uint8_t function_code,  uint8_t* modbus_data, uint8_t* response_buffer, uint8_t src);



#endif /* INC_MODBUS_H_ */
