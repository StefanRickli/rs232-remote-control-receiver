/*
 * misc.h
 *
 *  Created on: 10 mars 2016
 *      Author: faitaoudia
 */

//TODO Update with all used fkt RK

#ifndef MISC_MISC_H_
#define MISC_MISC_H_

// -----------------------------------------------------------------------
// Implementation of some useful functions
// -----------------------------------------------------------------------
/*
 * Convert a int to a string.
 */
char* itoa(int value, char* result, int base);

void panic(char[]);

// -----------------------------------------------------------------------------
// Some useful tool for manipulating circular buffer as FIFOs
// -----------------------------------------------------------------------------
#define FIFO_FULL(first, last, size)	( ((last + 1) == full) || ((first == 0) && (last == (size-1))) )
#define FIFO_EMPTY(first, last)			(first == last)
#define FIFO_INCR(p, size)				(p = ((p == (size-1)) ? 0 : p + 1))


#endif /* MISC_MISC_H_ */
