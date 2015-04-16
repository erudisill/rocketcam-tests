/*
 * omnivision_ov774.h
 *
 *  Created on: Apr 16, 2015
 *      Author: ericrudisill
 */

#ifndef INCLUDE_OMNIVISION_OV7740_H_
#define INCLUDE_OMNIVISION_OV7740_H_


#include <stdint.h>
#include "ov7740.h"


/** define a structure for omnivision register initialization values */
typedef struct _ov_reg
{
    /** Register to be written */
    uint8_t reg ;
    /** Value to be written in the register */
    uint8_t val ;
} ov_reg ;

#endif /* INCLUDE_OMNIVISION_OV7740_H_ */
