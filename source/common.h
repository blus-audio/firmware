// Copyright 2023 elagil
#ifndef SOURCE_COMMON_H_
#define SOURCE_COMMON_H_

#include <stddef.h>

#include "ch.h"
#include "inttypes.h"

/**
 * @brief Enable or disable the reporting thread (for debug purposes).
 */
#define ENABLE_REPORTING FALSE

/**
 * @brief Calculate the length (number of elements) of an array.
 *
 * @param _array The array, of which to calculate the length.
 */
#define ARRAY_LENGTH(_array) (sizeof(_array) / sizeof(_array[0]))

/**
 * @brief Wrap an unsigned number to a certain maximum value.
 *
 * @param value The value to wrap.
 * @param max_value The maximum value, at which wrapping occurs. The maximum
 * value itself wraps back to zero.
 * @return size_t The wrapped value.
 */
static inline size_t wrap_unsigned(size_t value, size_t max_value) {
    return value - ((value / max_value) * max_value);
}

/**
 * @brief Calculates the difference of unsigned values on a circular range of
 * values.
 * @details If the subtrahend is larger than the minuend, the result waps back
 * into the allowed range of values between zero and \a max_value - 1. The
 * overall result is wrapped by means of \a wrap_unsigned() .
 *
 * @param minuend The value from which to subtract.
 * @param subtrahend The value to subtract.
 * @param max_value The maximum value, at which wrapping occurs. See \a
 * wrap_unsigned() .
 * @return size_t The circular difference.
 */
static inline size_t subtract_circular_unsigned(size_t minuend,
                                                size_t subtrahend,
                                                size_t max_value) {
    if (subtrahend > minuend) {
        return wrap_unsigned(minuend + max_value - subtrahend, max_value);
    } else {
        return wrap_unsigned(minuend - subtrahend, max_value);
    }
}

/**
 * @brief Calculate the sum of unsigned values on a circular range of values.
 *
 * @param summand_a The first summand.
 * @param summand_b The second summand.
 * @param max_value The maximum value, at which wrapping occurs. See \a
 * wrap_unsigned() .
 * @return size_t The wrapped sum.
 */
static inline size_t add_circular_unsigned(size_t summand_a, size_t summand_b,
                                           size_t max_value) {
    return wrap_unsigned(summand_a + summand_b, max_value);
}

#endif  // SOURCE_COMMON_H_
