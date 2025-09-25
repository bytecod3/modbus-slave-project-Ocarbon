/**
 * @file relays.c
 * @brief Implements relay functions
 * @author Edwin
 */

#include "relays.h"


extern uint8_t RELAY_BANK_0;
extern uint8_t RELAY_BANK_1;
extern uint8_t RELAY_BANK_2;
extern uint8_t RELAY_BANK_3;


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

void relay_set(uint8_t relay_number) {
	uint8_t n = relay_resolve_bank(relay_number);
	if(n != -1) {


	}
}
