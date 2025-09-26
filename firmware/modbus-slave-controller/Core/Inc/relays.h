/*
 * @file relays.h
 * @brief declares relay variables and functions
 */

#ifndef INC_RELAYS_H_
#define INC_RELAYS_H_

#include "stm32f4xx_hal.h"
#include "custom_types.h"
#include "mcp23017.h"

#define EXPANDER1_BASE_ADDRESS 0x20
#define EXPANDER2_BASE_ADDRESS 0X21

#define NUM_RELAYS 	(32)
#define NUM_BANKS   (4)
#define RELAY_PORTB_OFFSET	(8)

#define BANK0_LIMIT (8)
#define BANK1_LIMIT (15)
#define BANK2_LIMIT (24)

#define RELAY_BANK_0  0
#define RELAY_BANK_1  1
#define RELAY_BANK_2  2
#define RELAY_BANK_3  3

#define PORTA 0
#define PORTB 1

#define EXPANDER_BANK_SIZE 		(16) // todo: move this to MCP23017 header file

extern I2C_HandleTypeDef hi2c1;


void relay_init();
void relay_set(uint8_t bank, uint8_t relay_num, uint8_t state);
boolean_t relay_read_state();
uint8_t relay_resolve_bank(uint8_t n);

uint8_t relay_read(uint8_t bank, uint8_t relay_num);
uint8_t relay_read_bank(uint8_t bank);

void relay_clear(uint8_t bank, uint8_t relay_num);
void relay_clear_bank(uint8_t bank);

#endif /* INC_RELAYS_H_ */
