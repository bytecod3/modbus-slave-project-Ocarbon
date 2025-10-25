/**
* @file custom_config.h
* @brief Project wide firmware settings
* @author Edwin Mwiti
* @date Sep 24 2025
*
*/

#ifndef INC_CUSTOM_CONFIG_H_
#define INC_CUSTOM_CONFIG_H_

#define MODBUS_TCP_ENABLE (0)

#define GET_INTERNAL_PARAMETERS 			1    ///< Set to 1 to get the internal CHIP parameters on the master device
#define USING_HEAP_4					1	///< by default this FREERTOS version uses heap 4 which implements get_heap_stats. I can use this for heap disgnostics


#endif /* INC_CUSTOM_CONFIG_H_ */
