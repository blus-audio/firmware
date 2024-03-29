// Copyright 2023 elagil

/**
 * @file
 * @brief   Commonly used helper functionality.
 *
 * @addtogroup common
 * @{
 */

#ifndef SOURCE_COMMON_H_
#define SOURCE_COMMON_H_

#include <stddef.h>

#include "ch.h"
#include "inttypes.h"

/**
 * @brief Calculate the length (number of elements) of an array.
 *
 * @param _array The array, of which to calculate the length.
 */
#define ARRAY_LENGTH(_array) (sizeof((_array)) / sizeof((_array)[0]))

/**
 * @brief Extract a byte from a provided value at a certain index.
 * @details The index is counted up from the LSB position.
 *
 * @param _value The value to extract a byte from.
 * @param _byte_index The byte index.
 */
#define GET_BYTE(_value, _byte_index) (((_value) >> (8u * (_byte_index))) & 0xFFu)

/**
 * @brief Swap upper and lower half-words (16 bit) of a word (32 bit).
 */
#define SWAP_HALF_WORDS(_value) ((((_value) >> 16u) & 0xFFFFu) | (((_value) << 16u) & 0xFFFFu))

/**
 * @brief Write a value to an array of bytes, starting at the LSB.
 *
 * @param p_bytes The array of bytes to write.
 * @param value The value to read from.
 * @param byte_count The number of bytes. Is clamped to \a sizeof(uint32_t) .
 */
__STATIC_INLINE void value_to_byte_array(uint8_t* p_bytes, uint32_t value, size_t byte_count) {
    uint32_t clamped_byte_count = byte_count;

    if (clamped_byte_count > sizeof(uint32_t)) {
        clamped_byte_count = sizeof(uint32_t);
    }

    for (size_t byte_index = 0; byte_index < clamped_byte_count; byte_index++) {
        p_bytes[byte_index] = GET_BYTE(value, byte_index);
    }
}

/**
 * @brief Write an array of bytes to a value, starting at the LSB.
 *
 * @param p_bytes The array of bytes to read.
 * @param p_value A pointer of the value to write.
 * @param byte_count The number of bytes. Is clamped to \a sizeof(uint32_t) .
 */
__STATIC_INLINE void byte_array_to_value(const uint8_t* p_bytes, uint32_t* p_value, size_t byte_count) {
    uint32_t clamped_byte_count = byte_count;

    if (clamped_byte_count > sizeof(uint32_t)) {
        clamped_byte_count = sizeof(uint32_t);
    }

    *p_value = 0u;

    for (size_t byte_index = 0; byte_index < clamped_byte_count; byte_index++) {
        *p_value |= (p_bytes[byte_index] << (8u * byte_index));
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
__STATIC_INLINE size_t wrap_unsigned(size_t value, size_t max_value) {
    chDbgAssert(max_value > 0, "Cannot wrap to a value of zero.");

    size_t temporary_value = value;

    while (temporary_value >= max_value) {
        temporary_value -= max_value;
    }

    return temporary_value;
}

/**
 * @brief Calculates the difference of unsigned values on a circular range of values.
 * @details If the subtrahend is larger than the minuend, the result waps back into the allowed range of values between
 * zero and \a max_value - 1. The overall result is wrapped by means of \a wrap_unsigned() .
 *
 * @param minuend The value from which to subtract.
 * @param subtrahend The value to subtract.
 * @param max_value The maximum value, at which wrapping occurs. See \a wrap_unsigned() .
 * @return size_t The circular difference.
 */
__STATIC_INLINE size_t subtract_circular_unsigned(size_t minuend, size_t subtrahend, size_t max_value) {
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
 * @param max_value The maximum value, at which wrapping occurs. See \a wrap_unsigned() .
 * @return size_t The wrapped sum.
 */
__STATIC_INLINE size_t add_circular_unsigned(size_t summand_a, size_t summand_b, size_t max_value) {
    return wrap_unsigned(summand_a + summand_b, max_value);
}

#endif  // SOURCE_COMMON_H_

/**
 * @}
 */
