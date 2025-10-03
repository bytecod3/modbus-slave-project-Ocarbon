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

#define MODBUS_RTU (0x00)
#define MODBUS_TCP (0x02)



uint16_t MODBUS_handle_function(uint8_t slave_id, uint8_t function_code,  uint8_t* modbus_data, uint8_t* response_buffer, uint8_t src);



#endif /* INC_MODBUS_H_ */
