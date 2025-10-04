/*
 * @file state_machine.h
 * @brief a simple state machine
 *
 * @date: Oct 4, 2025
 * @author eduh
 *
 *
 */

#ifndef SRC_STATE_MACHINE_H_
#define SRC_STATE_MACHINE_H_

/**
 * enum to hold possible device states
 */
typedef enum _state_machine {
	STATE_NOMINAL,
	STATE_FAULT
} Device_state_t ;


#endif /* SRC_STATE_MACHINE_H_ */
