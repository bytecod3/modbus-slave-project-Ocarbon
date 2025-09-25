/**
 * @file relays.c
 * @brief Implements relay functions
 * @author Edwin
 */

#include "relays.h"
#include "mcp23017.h"

// expander instances
MCP23017 exp1;
MCP23017 exp1;

MCP23017_instance expander1_inst = &exp1;
MCP23017_instance expander2_inst = &exp2;

uint8_t RELAY_BANK_0 = 0;
uint8_t RELAY_BANK_1 = 0;
uint8_t RELAY_BANK_2 = 0;
uint8_t RELAY_BANK_3 = 0;

uint8_t* RELAY_BANKS[NUM_BANKS] = {
		&RELAY_BANK_0,
		&RELAY_BANK_1,
		&RELAY_BANK_2,
		&RELAY_BANK_3
};

void relay_init() {

}

/**
 * @brief This function determines which bank the relay is located
 * @ret	Relay position in that bank (between 0 and 7)
 */
uint8_t relay_resolve_bank(uint8_t n) {
	if(n > NUM_RELAYS) {
		return -1; // todo: use error code
	} else {
		if( n < BANK0_LIMIT) { // using BANK 0 (0-7)
			return n;

		} else if(n > BANK0_LIMIT) { // using BANK 1 (8-15)
			return n % BANK0_LIMIT;

		} else if(n > BANK1_LIMIT) { // using BANK 2 (16 - 23)
			return n % BANK1_LIMIT;

		}  else if(n > BANK2_LIMIT) {
			return n % BANK2_LIMIT;
		} else {

			return -1; // handled
		}
	}

}

/**
 * @brief This function activates the given relay
 * It checks the BANK and position of relay in that BANK
 * Writes HIGH on that pin
 */

void relay_set(uint8_t bank, uint8_t relay_num, uint8_t state) {

	// check if the bank number and relay number are valid
	if (bank >= 4 || relay_num >= 8) return;

	switch (bank) {
		case RELAY_BANK_0:
			MCP_write_pin(expander1_inst, relay_num, state);
			break;

		case RELAY_BANK_1:
			MCP_write_pin(expander1_inst, relay_num + RELAY_PORTB_OFFSET, state);
			break;

		case RELAY_BANK_2:
			MCP_Write_pin(expander2_inst, relay_num, state);
			break;

		case RELAY_BANK_3:
			MCP_Write_pin(expander2_inst, relay_num + RELAY_PORTB_OFFSET, state);
			break;

	default:
		break;
	}


}
