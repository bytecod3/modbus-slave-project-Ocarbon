/*
 * @file max485.h
 * @brief This file declares MODBUS RTU functions
 * @date Sep 26, 2025
 * @author Edwin
 */

#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#include "stm32f4xx_hal.h"
#include "defines.h"

#define SLAVE_ID 0x01					///< This device's slave ID

/*
 * slave holding registers
 */
#define NUM_HOLDING_REGISTERS (10)
extern uint16_t holding_registers[];

#define firmware_version_holding_reg	(0x00) // maps to 40001
#define device_id_holding_reg			(0x02) // maps to 40003

/**
 * @brief This function create and initializes holding registers
 */
void modbus_rtu_init_holding_regs(uint16_t holding_regs[], int size);


/**
 * @brief typedef for MODBUS data
 */
typedef struct _modbus_packet {
	uint16_t len;
	uint8_t data[MODBUS_RTU_MAX_SIZE];
} ModBus_RTU_type_t;

// todo: create  pointer typedef

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
uint16_t MAX485_calculate_CRC(const uint8_t* buf, uint16_t len);

#endif /* INC_MODBUS_RTU_H_ */
