/*
 * @file relays.h
 * @brief declares relay variables and functions
 */

#ifndef INC_RELAYS_H_
#define INC_RELAYS_H_

#include "custom_types.h"

#define NUM_RELAYS 	(32)

#define BANK0_LIMIT (8)
#define BANK1_LIMIT (15)
#define BANK2_LIMIT (24)
#define BANK3_LIMIT ()

extern uint8_t RELAY_BANK_0;
extern uint8_t RELAY_BANK_1;
extern uint8_t RELAY_BANK_2;
extern uint8_t RELAY_BANK_3;

void relay_set(uint8_t relay_number);
void relay_clear(uint8_t relay_number);
boolean_t relay_read_state();
uint8_t relay_resolve_bank(uint8_t n);

#endif /* INC_RELAYS_H_ */
