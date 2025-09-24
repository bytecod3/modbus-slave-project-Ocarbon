/**
* @file custom_types.h
* @brief My defined data types for various data related stuff
* @author Edwin Mwiti
* @date Sep 24 2025
*
*/


#ifndef INC_CUSTOM_TYPES_H_
#define INC_CUSTOM_TYPES_H_

#include "cmsis_os.h"
#include "custom_config.h"

#if GET_INTERNAL_PARAMETERS
typedef struct chip_parameters {
	uint32_t uid[3];
	float die_temperature;
	uint32_t core_frequency;
	float ref_voltage;
	float vcc; // TODO
} chip_parameters_t;

#endif

typedef struct diagnostics {
#if GET_INTERNAL_PARAMETERS
	chip_parameters_t chip_parameters;
#endif

	size_t free_heap_size;
	size_t minimum_ever_free_heap_size;

//#if USING_HEAP_4
//	Heap
//#endif



} diagnostics_type_t;


#endif /* INC_CUSTOM_TYPES_H_ */
