/*
 * @file max485.h
 * @brief This file declares MODBUS RTU functions
 * @date Sep 26, 2025
 * @author  Edwin
 */

#ifndef INC_MAX485_H_
#define INC_MAX485_H_

#include "stm32f4xx_hal.h"

/**
 * @brief defines a struct for a MAX485 transceiver
 */
typedef struct {
	UART_HandleTypeDef* uart_instance;
	GPIO_TypeDef* DE_RE_PORT;
	uint16_t DE_RE_pin;


} MAX485;

typedef MAX485* MAX485_instance;

void MAX485_init(MAX485_instance inst, uint8_t DE_RE_pin);
void MAX485_enable_transmit(MAX485_instance inst);
void MAX485_enable_receive(MAX485_instance inst);
void MAX485_send(MAX485_instance inst, uint8_t* data, uint16_t len, uint32_t timeout);
void MAX485_receive(MAX485_instance inst, uint8_t* data, uint16_t len, uint32_t timeout);

#endif /* INC_MAX485_H_ */
