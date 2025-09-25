/*
 * @file relays.h
 * @brief declares relay variables and functions
 */

#ifndef INC_RELAYS_H_
#define INC_RELAYS_H_

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
#define BANK3_LIMIT ()

extern uint8_t RELAY_BANK_0;
extern uint8_t RELAY_BANK_1;
extern uint8_t RELAY_BANK_2;
extern uint8_t RELAY_BANK_3;

#define PORTA 0
#define PORTB 1

uint8_t* RELAY_BANKS[4];

#define EXPANDER_BANK_SIZE 		(16) // todo: move this to MCP23017 header file


// expander instances
extern


void relay_init();
void relay_set(uint8_t bank, uint8_t relay_num, uint8_t state)
void relay_clear(uint8_t relay_number);
boolean_t relay_read_state();
uint8_t relay_resolve_bank(uint8_t n);

uint8_t relay_read_single_bank(uint8_t bank, uint8_t relay_num);
uint32_t relay_read_all_banks();

#endif /* INC_RELAYS_H_ */
