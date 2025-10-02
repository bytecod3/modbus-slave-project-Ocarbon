/*
 * @file modbus_tcp.c
 * @brief Describes function to be used for ModBus TCP
 * @date Oct 2, 2025
 * @author Edwin
 *
 */

#include "modbus_tcp.h"


/**
 * @brief This function handles the received modbus PDU tyoe
 * @return length of response
 */
uint16_t modbus_tcp_handle(uint8_t function_code, uint8_t* pdu_data, uint16_t pdu_length, uint8_t* response) {
	switch(function_code) {
	case 0x01:
		// read coil
		break;
	case 0x03:
		// read holding registers
		break;
	default:
		// handle exception
	}
}


