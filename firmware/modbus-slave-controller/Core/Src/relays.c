/**
 * @file relays.c
 * @brief Implements relay functions
 * @author Edwin
 */

#include "mcp23017.h"
#include "relays.h"

/**
 * I want to expose a clean RELAY drover API so I use a static expander object
 * that is visible in this file
 */
static MCP23017 exp1;
static MCP23017 exp2;

MCP23017_instance expander1_inst = &exp1;
MCP23017_instance expander2_inst = &exp2;


/**
 * @brief this function initialize all the relay pins as outputs
 * @param none
 */
void relay_init() {

	// initialize the expander
	// Note: teh address is hardware configured using A0,A1,A2 pins on the expander chip
	MCP_initialize(expander1_inst, &hi2c1, EXPANDER1_BASE_ADDRESS);
	MCP_initialize(expander1_inst, &hi2c1, EXPANDER1_BASE_ADDRESS);

	// set all pins as output
	MCP_all_pinmode(expander1_inst, HIGH, PORTA);
	MCP_all_pinmode(expander1_inst, HIGH, PORTB);
	MCP_all_pinmode(expander2_inst, HIGH, PORTA);
	MCP_all_pinmode(expander2_inst, HIGH, PORTB);
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
			MCP_write_pin(expander2_inst, relay_num, state);
			break;

		case RELAY_BANK_3:
			MCP_write_pin(expander2_inst, relay_num + RELAY_PORTB_OFFSET, state);
			break;

	default:
		break; // todo: handle default case
	}
}

/**
 * @brief This function will read the state of a single relay in a given bank
 */
uint8_t relay_read(uint8_t bank, uint8_t relay_num) {
	if(bank >=4 || relay_num >= 8) return;

	switch (bank) {
		case RELAY_BANK_0:
			MCP_read_pin(expander1_inst, relay_num);
			break;

		case RELAY_BANK_1:
			MCP_read_pin(expander1_inst, relay_num + RELAY_PORTB_OFFSET);
			break;

		case RELAY_BANK_2:
			MCP_read_pin(expander2_inst, relay_num);
			break;

		case RELAY_BANK_3:
			MCP_read_pin(expander2_inst, relay_num + RELAY_PORTB_OFFSET);
			break;

		default:
			break; // todo: handle default case
	}
}

/**
 * @brief This function will read a single bank of relays
 * @oaram bank which bank to read from
 * @return bank_val uin8_t with relay states (1 for set, 0 for not set)
 *
 */
uint8_t relay_read_bank(uint8_t bank) {
	if(bank >= 4) return;

	uint8_t bank_val;

	switch (bank) {
		case RELAY_BANK_0:
			bank_val = MCP_read_port(expander1_inst, PORTA);
			break;

		case RELAY_BANK_1:
			bank_val = MCP_read_port(expander1_inst, PORTB);
			break;

		case RELAY_BANK_2:
			bank_val = MCP_read_port(expander2_inst, PORTA);
		    break;

		case RELAY_BANK_3:
			bank_val = MCP_read_port(expander2_inst, PORTB);
			break;

		default:
			break; // todo: handle default case
	}

	return bank_val;

}

/**
 * @brief This function clears a single relay in the chosen bank
 * @param bank Bank to clear the relay from
 * @param relay_num	relay number to clear (0 - 7)
 */
void relay_clear(uint8_t bank, uint8_t relay_num) {
	if(bank >=4 || relay_num >= 8) return;

	switch (bank) {
		case RELAY_BANK_0:
			MCP_write_pin(expander1_inst, relay_num, LOW);
			break;

		case RELAY_BANK_1:
			MCP_write_pin(expander1_inst, relay_num + RELAY_PORTB_OFFSET, LOW);
			break;

		case RELAY_BANK_2:
			MCP_write_pin(expander2_inst, relay_num, LOW);
			break;

		case RELAY_BANK_3:
			MCP_write_pin(expander2_inst, relay_num + RELAY_PORTB_OFFSET, LOW);
			break;

		default:
			break; // todo: handle default case
	}

}

/**
 * @brief This function clears a whole bank at once hence resetting all the relays in that bank
 * @param bank the bank to reset
 */
void relay_clear_bank(uint8_t bank) {
	if(bank >=4) return;

	switch (bank) {
		case RELAY_BANK_0:
			MCP_clear_port(expander1_inst, PORTA);
			break;

		case RELAY_BANK_1:
			MCP_clear_port(expander1_inst, PORTB);
			break;

		case RELAY_BANK_2:
			MCP_clear_port(expander1_inst, PORTA);
			break;

		case RELAY_BANK_3:
			MCP_clear_port(expander1_inst, PORTB);
			break;

		default:
			break; // todo: handle default case
	}

}
