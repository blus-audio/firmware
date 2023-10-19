// Copyright 2023 elagil

/**
 * @file
 * @brief   Printing functionality.
 *
 * @addtogroup common
 * @{
 */

#ifndef SOURCE_PRINT_H_
#define SOURCE_PRINT_H_

#include <stdarg.h>

#include "chprintf.h"
#include "common.h"

/**
 * @brief Generic print function, which can perform different functions.
 *
 * @param fmt Format specifier.
 * @param ... Additional arguments.
 */
static inline void PRINTF(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    chvprintf((BaseSequentialStream*)&SD2, fmt, args);
    va_end(args);
}

#endif  // SOURCE_PRINT_H_
