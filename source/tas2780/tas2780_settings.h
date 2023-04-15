// Copyright 2023 elagil
#ifndef SOURCE_TAS2780_TAS2780_SETTINGS_H_
#define SOURCE_TAS2780_TAS2780_SETTINGS_H_

#include "tas2780.h"

/**
 * @brief The number of amplifiers.
 */
#define TAS2780_DEVICE_COUNT (4u)

/**
 * @brief Device addresses.
 */
static const uint8_t TAS2780_DEVICE_ADDRESSES[TAS2780_DEVICE_COUNT] = {0x72u, 0x74u, 0x7Au, 0x7Cu};

/**
 * @brief Device channels.
 */
static const enum tas2780_channel TAS2780_DEVICE_CHANNELS[TAS2780_DEVICE_COUNT] = {
    TAS2780_CHANNEL_LEFT, TAS2780_CHANNEL_RIGHT, TAS2780_CHANNEL_LEFT, TAS2780_CHANNEL_RIGHT};

/**
 * @brief The TDM slot indices, on which the amplifiers shall play.
 */
static const uint8_t TAS2780_TDM_SLOT_INDICES[TAS2780_DEVICE_COUNT] = {0u, 1u, 2u, 3u};

#endif  // SOURCE_TAS2780_TAS2780_SETTINGS_H_
