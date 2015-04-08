/*
 * cpdefs.h
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPDEFS_H_
#define SRC_CPDEFS_H_

// Requires .strings section in linker script, with KEEP so it isn't removed
// Placing the .strings section in RAM will provide quicker access but use up more memory.
// Placing the .strings section in ROM will maximize memory
#define CPSTRCONST			char * const __attribute((used, section(".strings")))
#define CPSTRCONSTPTR		char const *

#endif /* SRC_CPDEFS_H_ */
