#ifndef _COMMON_H_
#define _COMMON_H_

/**
 * @brief Calculate the length (number of elements) of an array.
 *
 * @param _array The array, of which to calculate the length.
 */
#define ARRAY_LENGTH(_array) (sizeof(_array) / sizeof(_array[0]))

#endif // _COMMON_H_
