/*
 * mcp23017.h
 *
 *@brief MCP23017 IO expander driver
 *  Created on: Sep 25, 2025
 *      Author: eduh
 */

#ifndef INC_MCP23017_H_
#define INC_MCP23017_H_

#include "stm32f4xx_hal.h"

#define INPUT 		(0)
#define OUTPUT 		(1)
#define LOW 		(0)
#define HIGH 		(1)

// register addresses
#define IODIRA        0x00
#define IOPOLA        0x02
#define GPINTENA      0x04
#define DEFVALA       0x06
#define INTCONA       0x08
#define IOCON         0x0A
#define GPPUA         0x0C
#define INTFA         0x0E
#define INTCAPA       0x10
#define MCP_GPIOA     0x12
#define OLATA 		  0x14

#define  IODIRB      0x01
#define  IOPOLB      0x03
#define  GPINTENB    0x05
#define  DEFVALB     0x07
#define  INTCONB     0x09
#define  GPPUB       0x0D
#define  INTFB       0x0F
#define  INTCAPB     0x11
#define  MCP_GPIOB   0x13
#define  OLATB       0x15

typedef struct _MCP23017{
	uint8_t address;
	I2C_HandleTypeDef* i2c_handle;
} MCP23017;

/* instance of MCP23017 expander*/
typedef MCP23017* MCP23017_instance;


void MCP_initialize(MCP23017_instance inst, I2C_HandleTypeDef* i2c_handle, uint8_t address);
void MCP_pinmode(MCP23017_instance inst, uint8_t pin, uint8_t mode);
void MCP_all_pinmode(MCP23017_instance inst, uint8_t state);
void MCP_write_pin(MCP23017_instance inst, uint8_t pin, uint8_t level);
uint8_t MCP_read_pin(MCP23017_instance inst, uint8_t pin);
uint8_t MCP_read_port(MCP23017_instance inst, uint8_t port_num);


#endif /* INC_MCP23017_H_ */
