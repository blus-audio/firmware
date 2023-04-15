// Copyright 2023 elagil
#ifndef SOURCE_COMMON_H_
#define SOURCE_COMMON_H_

#include <stddef.h>

#include "ch.h"
#include "inttypes.h"

/**
 * @brief Enable or disable the reporting thread (for debug purposes).
 * @note Set to \a TRUE or \a FALSE .
 */
#define ENABLE_REPORTING TRUE

/**
 * @brief Calculate the length (number of elements) of an array.
 *
 * @param _array The array, of which to calculate the length.
 */
#define ARRAY_LENGTH(_array) (sizeof(_array) / sizeof(_array[0]))

/**
 * @brief Extract a byte from a provided value at a certain index.
 * @details The index is counted up from the LSB position.
 *
 * @param _value The value to extract a byte from.
 * @param _byte_index The byte index.
 */
#define GET_BYTE(_value, _byte_index) ((_value >> (8u * _byte_index)) & 0xFFu)

/**
 * @brief Write a value to an array of bytes, starting at the LSB.
 *
 * @param p_bytes The array of bytes to write.
 * @param value The value to read from.
 * @param byte_count The number of bytes. Is clamped to \a sizeof(uint32_t) .
 */
static inline void value_to_byte_array(uint8_t* p_bytes, uint32_t value,
                                       size_t byte_count) {
    uint32_t clamped_byte_count = byte_count;

    if (clamped_byte_count > sizeof(uint32_t)) {
        clamped_byte_count = sizeof(uint32_t);
    }

    for (size_t byte_index = 0; byte_index < clamped_byte_count; byte_index++) {
        p_bytes[byte_index] = GET_BYTE(value, byte_index);
    }
}

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
