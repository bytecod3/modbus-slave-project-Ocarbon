/*
 * @file modbus_tcp.h
 * @brief declares function to be used for ModBus TCP
 * @date Oct 2, 2025
 * @author Edwin
 *
 */

#ifndef INC_MODBUS_TCP_H_
#define INC_MODBUS_TCP_H_

#include "stm32f4xx_hal.h"
#include "defines.h"

/**
 * Type to hold modbus TCP packet
 */
typedef struct _modbus_tcp_packet {
	uint16_t len;
	uint8_t data[MODBUS_TCP_MAX_SIZE];
} Modbus_tcp_type_t;


#endif /* INC_MODBUS_TCP_H_ */
