// Header for project utility functions

#ifndef _UTILITY_H_
#define _UTILITY_H_

// Conversion function that takes a 32-bit Integer (int) and fills a provided array of 4 8-bit Integers (char)
void int_to_char4 ( char *arr, int n );

// Conversion function that takes a 32-bit Integer (unsigned int) and fills a provided array of 4 8-bit Integers (char)
void uint_to_char4 ( char *arr, unsigned int n );

// Conversion function that takes a pointer to an array of 4 8-bit Integers (char) and returns a 32-bit Integer (int)
int char4_to_int ( char *arr );

// Conversion function that takes a pointer to an array of 4 8-bit Integers (char) and returns a 32-bit Integer (unsigned int)
unsigned int char4_to_uint ( char *arr );

#endif // _UTILITY_H_
