/*
 * max485.c
 *
 *  Created on: Sep 26, 2025
 *      Author: eduh
 */

#include "max485.h"


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
	HAL_GPIO_
}
