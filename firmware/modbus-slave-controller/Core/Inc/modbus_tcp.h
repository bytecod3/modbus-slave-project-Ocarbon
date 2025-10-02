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

/**
 * @brief This function handles the received modbus PDU tyoe
 */
uint16_t modbus_tcp_handle(uint8_t function_code, uint8_t* pdu_data, uint16_t pdu_length, uint8_t* response);

/**
 * @brief This function responds back to TCP master
 */
void modbus_tcp_send_response();

#endif /* INC_MODBUS_TCP_H_ */
